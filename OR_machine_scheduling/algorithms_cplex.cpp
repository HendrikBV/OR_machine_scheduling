#include "algorithms.h"
#include <stdexcept>
#include <memory>
#include <iostream>
#include <chrono>
#include <queue>



namespace MS
{

	void CPLEXIP::run(bool verbose)
	{
		_output_screen = verbose;
		_output.set_on(verbose);

		initialize_cplex();
		build_problem();
		solve_problem();
		clear_cplex();
	}


	void CPLEXIP::initialize_cplex()
	{
		int status = 0;
		char error_text[CPXMESSAGEBUFSIZE];

		// open the CPLEX environment
		env = CPXopenCPLEX(&status);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::initialize_cplex(). \nCouldn't open CPLEX. \nReason: " + std::string(error_text));
		}

		// turn output to screen on/off
		if (_output_screen)
			status = CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_ON);
		else
			status = CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_OFF);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::initialize_cplex(). \nCouldn't change param SCRIND. \nReason: " + std::string(error_text));
		}
	}


	void CPLEXIP::build_problem()
	{
		char error_text[CPXMESSAGEBUFSIZE];
		int status = 0;
		double obj[1];			// Objective function
		double lb[1];			// Lower bound variables
		double ub[1];			// Upper bound variables
		double rhs[1];			// Right-hand side constraints
		char sense[1];			// Sign of constraint
		char type[1];			// Type of variable (integer, binary, fractional)
		int nonzeroes = 0;		// To calculate number of nonzero coefficients in each constraint
		int matbeg[1];			// Begin position of the constraint
		std::unique_ptr<int[]> matind; // Position of each element in constraint matrix
		std::unique_ptr<double[]> matval; // Value of each element in constraint matrix

		matbeg[0] = 0;


		// allocate memory
		const size_t maxnonzeroes = 100000;
		matind = std::make_unique<int[]>(maxnonzeroes);
		matval = std::make_unique<double[]>(maxnonzeroes);


		// create the problem
		problem = CPXcreateprob(env, &status, "IP_model_type1");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't create problem. \nReason: " + std::string(error_text));
		}

		// problem is minimization
		status = CPXchgobjsen(env, problem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change objective to minimization. \nReason: " + std::string(error_text));
		}


		// add variables
		const size_t nbjobs = _jobs.size();
		int nb_variables = -1;

		// variables x_jk
		for (size_t j = 0; j < nbjobs; ++j)
		{
			for (size_t k = 0; k < nbjobs; ++k)
			{
				++nb_variables;

				obj[0] = 0;
				lb[0] = 0;
				ub[0] = 1;
				type[0] = 'B';

				status = CPXnewcols(env, problem, 1, obj, lb, ub, type, NULL);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
				}

				// change variable name
				std::string varname = "x_" + std::to_string(j + 1) + "_" + std::to_string(k + 1);
				status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
				}
			}
		}

		// variables C_k
		for (size_t k = 0; k < nbjobs; ++k)
		{
			++nb_variables;

			obj[0] = 0;
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
			}

			// change variable name
			std::string varname = "C_" + std::to_string(k + 1);
			status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
			}
		}

		// variables T_j
		for (size_t j = 0; j < nbjobs; ++j)
		{
			++nb_variables;

			obj[0] = 1;
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
			}

			// change variable name
			std::string varname = "T_" + std::to_string(j + 1);
			status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
			}
		}



		// lambdas to access variables
		auto index_x_jk = [nbjobs](int j, int k) -> int {
			return j * nbjobs + k;
			};

		auto index_C_k = [nbjobs](int k) -> int {
			return nbjobs * nbjobs + k;
			};

		auto index_T_j = [nbjobs](int j) -> int {
			return nbjobs * nbjobs + nbjobs + j;
			};



		// add constraints
		int nb_constraints = -1;

		// 1: every job in exactly one position in the sequence
		for (int j = 0; j < nbjobs; ++j)
		{
			++nb_constraints;

			rhs[0] = 1;
			sense[0] = 'E';
			matbeg[0] = 0;

			nonzeroes = 0;

			// x_jk
			for (int k = 0; k < nbjobs; ++k)
			{
				matind[nonzeroes] = index_x_jk(j, k);
				matval[nonzeroes] = 1;
				++nonzeroes;
			}

			if (nonzeroes >= maxnonzeroes)
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

			status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
			}

			// change name of constraint
			std::string conname = "c1_" + std::to_string(j + 1);
			status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
			}
		}

		// 2: every position in the sequence has exactly one job
		for (int k = 0; k < nbjobs; ++k)
		{
			++nb_constraints;

			rhs[0] = 1;
			sense[0] = 'E';
			matbeg[0] = 0;

			nonzeroes = 0;

			// x_jk
			for (int j = 0; j < nbjobs; ++j)
			{
				matind[nonzeroes] = index_x_jk(j, k);
				matval[nonzeroes] = 1;
				++nonzeroes;
			}

			if (nonzeroes >= maxnonzeroes)
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

			status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
			}

			// change name of constraint
			std::string conname = "c2_" + std::to_string(k + 1);
			status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
			}
		}

		// 3 & 4: completion time job in position k
		for (int k = 0; k < nbjobs; ++k)
		{
			++nb_constraints;

			rhs[0] = 0;
			sense[0] = 'G';
			matbeg[0] = 0;

			nonzeroes = 0;

			// x_jk
			for (int j = 0; j < nbjobs; ++j)
			{
				matind[nonzeroes] = index_x_jk(j, k);
				matval[nonzeroes] = -_jobs[j].duration;
				++nonzeroes;
			}

			// C_k
			{
				matind[nonzeroes] = index_C_k(k);
				matval[nonzeroes] = 1;
				++nonzeroes;
			}

			// C_k-1
			if (k >= 1)
			{
				matind[nonzeroes] = index_C_k(k - 1);
				matval[nonzeroes] = -1;
				++nonzeroes;
			}

			if (nonzeroes >= maxnonzeroes)
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

			status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
			}

			// change name of constraint
			std::string conname = "c3and4_" + std::to_string(k + 1);
			status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
			}
		}

		// 5: Tardiness job j
		int BigM = 0;
		for (auto&& jj : _jobs)
			BigM += jj.duration;

		for (int j = 0; j < nbjobs; ++j)
		{
			for (int k = 0; k < nbjobs; ++k)
			{
				++nb_constraints;

				rhs[0] = -_jobs[j].due_date - BigM;
				sense[0] = 'G';
				matbeg[0] = 0;

				nonzeroes = 0;

				// x_jk
				{
					matind[nonzeroes] = index_x_jk(j, k);
					matval[nonzeroes] = -BigM;
					++nonzeroes;
				}

				// C_k
				{
					matind[nonzeroes] = index_C_k(k);
					matval[nonzeroes] = -1;
					++nonzeroes;
				}

				// T_j
				{
					matind[nonzeroes] = index_T_j(j);
					matval[nonzeroes] = 1;
					++nonzeroes;
				}

				if (nonzeroes >= maxnonzeroes)
					throw std::runtime_error("Error in function CPLEXIP::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

				status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
				}

				// change name of constraint
				std::string conname = "c5_" + std::to_string(k + 1);
				status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
				}
			}
		}



		// write to file
		status = CPXwriteprob(env, problem, "CPLEXIP.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't write problem to lp-file. \nReason: " + std::string(error_text));
		}
	}


	void CPLEXIP::solve_problem()
	{
		char error_text[CPXMESSAGEBUFSIZE];
		int status = 0;
		int solstat = 0;
		std::unique_ptr<double[]> solution_problem;
		double objval;

		/*status = CPXreadcopyprob(env, problem, "CPLEXIP.lp", NULL); /// Dit werkt wel, maar probleem rechtstreeks geeft error
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::solve_problem(). \nCouldn't read problem from lp-file. \nReason: " + std::string(error_text));
		}*/

		// Set allowed computation time
		status = CPXsetdblparam(env, CPXPARAM_TimeLimit, _max_computation_time);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IP_model_allocation::solve_problem(). \nCouldn't set time limit. \nReason: " + std::string(error_text));
		}

		// Assign memory for solution
		const int numvar = CPXgetnumcols(env, problem);
		solution_problem = std::make_unique<double[]>(numvar);

		// Optimize the problem
		std::cout << "\nUsing an IP model with x_jk = 1 if job j is at position k in the sequence, 0 otherwise"
			<< "\nUsing CPLEX to solve the model ...\n";
		auto start_time = std::chrono::system_clock::now();


		status = CPXmipopt(env, problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time_IP = std::chrono::system_clock::now() - start_time;


		// Get the solution
		status = CPXsolution(env, problem, &solstat, &objval, solution_problem.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
		}

		char solstat_text[CPXMESSAGEBUFSIZE];
		auto p = CPXgetstatstring(env, solstat, solstat_text);
		if (p != nullptr)
		{
			std::cout << "\nResult solve: " << solstat_text;

			if (solstat == CPXMIP_OPTIMAL || solstat == CPXMIP_OPTIMAL_TOL || solstat == CPXMIP_TIME_LIM_FEAS)
			{
				_best_value = objval + 0.0001;

				_best_sequence.clear();
				_best_sequence.reserve(_jobs.size());

				for (int k = 0; k < _jobs.size(); ++k) {
					int job = 0;
					for (int j = 0; j < _jobs.size(); ++j) {
						if (solution_problem[j * _jobs.size() + k] > 0.99) {
							job = j;
							break;
						}
					}
					_best_sequence.push_back(job);
				}


				std::cout << "\nElapsed time (s): " << elapsed_time_IP.count();

				std::cout << "\nObjective value = " << _best_value;
				std::cout << "\nSequence: ";
				for (auto&& job : _best_sequence)
					std::cout << job + 1 << " ";
			}
		}

	}


	void CPLEXIP::clear_cplex()
	{
		int status = 0;
		char error_text[CPXMESSAGEBUFSIZE];

		// Free the problem
		status = CPXfreeprob(env, &problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::clear_cplex(). \nCouldn't free problem. \nReason: " + std::string(error_text));
		}

		// Close the cplex environment
		status = CPXcloseCPLEX(&env);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::clear_cplex(). \nCouldn't close cplex environment. \nReason: " + std::string(error_text));
		}
	}


	///////////////////////////////////////////////////////////////////////////


	void CPLEXIPAlt::run(bool verbose)
	{
		_output_screen = verbose;
		_output.set_on(verbose);

		initialize_cplex();
		build_problem();
		solve_problem();
		clear_cplex();
	}


	void CPLEXIPAlt::initialize_cplex()
	{
		int status = 0;
		char error_text[CPXMESSAGEBUFSIZE];

		// open the CPLEX environment
		env = CPXopenCPLEX(&status);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::initialize_cplex(). \nCouldn't open CPLEX. \nReason: " + std::string(error_text));
		}

		// turn output to screen on/off
		if (_output_screen)
			status = CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_ON);
		else
			status = CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_OFF);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::initialize_cplex(). \nCouldn't change param SCRIND. \nReason: " + std::string(error_text));
		}
	}


	void CPLEXIPAlt::build_problem()
	{
		char error_text[CPXMESSAGEBUFSIZE];
		int status = 0;
		double obj[1];			// Objective function
		double lb[1];			// Lower bound variables
		double ub[1];			// Upper bound variables
		double rhs[1];			// Right-hand side constraints
		char sense[1];			// Sign of constraint
		char type[1];			// Type of variable (integer, binary, fractional)
		int nonzeroes = 0;		// To calculate number of nonzero coefficients in each constraint
		int matbeg[1];			// Begin position of the constraint
		std::unique_ptr<int[]> matind; // Position of each element in constraint matrix
		std::unique_ptr<double[]> matval; // Value of each element in constraint matrix

		matbeg[0] = 0;


		// allocate memory
		const size_t maxnonzeroes = 100000;
		matind = std::make_unique<int[]>(maxnonzeroes);
		matval = std::make_unique<double[]>(maxnonzeroes);


		// create the problem
		problem = CPXcreateprob(env, &status, "IP_model_type1");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't create problem. \nReason: " + std::string(error_text));
		}

		// problem is minimization
		status = CPXchgobjsen(env, problem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change objective to minimization. \nReason: " + std::string(error_text));
		}


		// add variables
		const size_t nbjobs = _jobs.size();
		int nb_variables = -1;

		// variables z_ij
		for (size_t i = 0; i < nbjobs; ++i)
		{
			for (size_t j = 0; j < nbjobs; ++j)
			{
				++nb_variables;

				obj[0] = 0;
				lb[0] = 0;
				ub[0] = 1;
				type[0] = 'B';

				status = CPXnewcols(env, problem, 1, obj, lb, ub, type, NULL);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
				}

				// change variable name
				std::string varname = "z_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
				status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
				}
			}
		}

		// variables T_j
		for (size_t j = 0; j < nbjobs; ++j)
		{
			++nb_variables;

			obj[0] = 1;
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
			}

			// change variable name
			std::string varname = "T_" + std::to_string(j + 1);
			status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
			}
		}



		// lambdas to access variables
		auto index_z_ij = [nbjobs](int i, int j) -> int {
			return i * nbjobs + j;
			};

		auto index_T_j = [nbjobs](int j) -> int {
			return nbjobs * nbjobs + j;
			};



		// add constraints
		int nb_constraints = -1;

		// 1: z_ij + z_ji = 1
		for (int i = 0; i < nbjobs; ++i)
		{
			for (int j = 0; j < nbjobs; ++j)
			{
				if (i < j)
				{
					++nb_constraints;

					rhs[0] = 1;
					sense[0] = 'E';
					matbeg[0] = 0;

					nonzeroes = 0;

					// z_ij
					{
						matind[nonzeroes] = index_z_ij(i, j);
						matval[nonzeroes] = 1;
						++nonzeroes;
					}

					// z_ji
					{
						matind[nonzeroes] = index_z_ij(j, i);
						matval[nonzeroes] = 1;
						++nonzeroes;
					}

					if (nonzeroes >= maxnonzeroes)
						throw std::runtime_error("Error in function CPLEXIP::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

					status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
					}

					// change name of constraint
					std::string conname = "c1_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
					status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
					}
				}
			}
		}

		// 2: z_ij + z_jk - z_ik <= 1
		for (int i = 0; i < nbjobs; ++i)
		{
			for (int j = 0; j < nbjobs; ++j)
			{
				for (int k = 0; k < nbjobs; ++k)
				{
					if (i != j && i != k && j != k)
					{
						++nb_constraints;

						rhs[0] = 1;
						sense[0] = 'L';
						matbeg[0] = 0;

						nonzeroes = 0;

						// z_ij
						{
							matind[nonzeroes] = index_z_ij(i, j);
							matval[nonzeroes] = 1;
							++nonzeroes;
						}

						// z_jk
						{
							matind[nonzeroes] = index_z_ij(j, k);
							matval[nonzeroes] = 1;
							++nonzeroes;
						}

						// -z_ik
						{
							matind[nonzeroes] = index_z_ij(i, k);
							matval[nonzeroes] = -1;
							++nonzeroes;
						}

						if (nonzeroes >= maxnonzeroes)
							throw std::runtime_error("Error in function CPLEXIP::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

						status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
						}

						// change name of constraint
						std::string conname = "c2_" + std::to_string(i + 1) + "_" + std::to_string(j + 1) + "_" + std::to_string(k + 1);
						status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}

		// 3: T_j - sum(i) p_i z_ij >= p_j - d_j   forall j
		for (int j = 0; j < nbjobs; ++j)
		{
			++nb_constraints;

			rhs[0] = _jobs[j].duration - _jobs[j].due_date;
			sense[0] = 'G';
			matbeg[0] = 0;

			nonzeroes = 0;

			// z_ij
			for (int i = 0; i < nbjobs; ++i)
			{
				matind[nonzeroes] = index_z_ij(i, j);
				matval[nonzeroes] = -_jobs[i].duration;
				++nonzeroes;
			}

			// T_j
			{
				matind[nonzeroes] = index_T_j(j);
				matval[nonzeroes] = 1;
				++nonzeroes;
			}

			if (nonzeroes >= maxnonzeroes)
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

			status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
			}

			// change name of constraint
			std::string conname = "c3_" + std::to_string(j + 1);
			status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
			}
		}



		// write to file
		status = CPXwriteprob(env, problem, "CPLEXIPAlt.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::build_problem(). \nCouldn't write problem to lp-file. \nReason: " + std::string(error_text));
		}
	}


	void CPLEXIPAlt::solve_problem()
	{
		char error_text[CPXMESSAGEBUFSIZE];
		int status = 0;
		int solstat = 0;
		std::unique_ptr<double[]> solution_problem;
		double objval;

		// Set allowed computation time
		status = CPXsetdblparam(env, CPXPARAM_TimeLimit, _max_computation_time);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IP_model_allocation::solve_problem(). \nCouldn't set time limit. \nReason: " + std::string(error_text));
		}

		// Assign memory for solution
		const int numvar = CPXgetnumcols(env, problem);
		solution_problem = std::make_unique<double[]>(numvar);

		// Optimize the problem
		std::cout << "\nUsing an IP model with z_ij = 1 if job i is scheduled before job j, 0 otherwise"
			<< "\nUsing CPLEX to solve the model ...\n";
		auto start_time = std::chrono::system_clock::now();


		status = CPXmipopt(env, problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time_IP = std::chrono::system_clock::now() - start_time;


		// Get the solution
		status = CPXsolution(env, problem, &solstat, &objval, solution_problem.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
		}

		char solstat_text[CPXMESSAGEBUFSIZE];
		auto p = CPXgetstatstring(env, solstat, solstat_text);
		if (p != nullptr)
		{
			std::cout << "\nResult solve: " << solstat_text;

			if (solstat == CPXMIP_OPTIMAL || solstat == CPXMIP_OPTIMAL_TOL || solstat == CPXMIP_TIME_LIM_FEAS)
			{
				_best_value = objval + 0.0001;

				_best_sequence.clear();
				_best_sequence.reserve(_jobs.size());

				{
					auto n = _jobs.size();  // Number of jobs
					std::vector<int> inDegree(n, 0);  // Number of jobs that must come before each job
					std::vector<std::vector<int>> graph(n);  // Adjacency list for edges i -> j

					// Build the graph and compute in-degrees
					for (int i = 0; i < n; ++i) {
						for (int j = 0; j < n; ++j) {
							if (i != j && solution_problem[i * n + j] > 0.99) {  // i precedes j
								graph[i].push_back(j);
								inDegree[j]++;
							}
						}
					}

					// Find jobs with no predecessors (in-degree = 0)
					std::queue<int> queue;
					for (int i = 0; i < n; ++i) {
						if (inDegree[i] == 0) {
							queue.push(i);
						}
					}

					// Topological sort to get the sequence
					while (!queue.empty()) {
						int job = queue.front();
						queue.pop();
						_best_sequence.push_back(job);

						// Reduce in-degree of all jobs that follow 'job'
						for (int next : graph[job]) {
							inDegree[next]--;
							if (inDegree[next] == 0) {
								queue.push(next);
							}
						}
					}

					// Check if we got a valid sequence (length should be n)
					if (_best_sequence.size() != n) {
						std::cerr << "Error: Invalid or cyclic sequence!" << std::endl;
					}
				}


				/*for (int k = 0; k < _jobs.size(); ++k) {
					int job = 0;
					for (int j = 0; j < _jobs.size(); ++j) {
						if (solution_problem[j * _jobs.size() + k] > 0.99) {
							job = j;
							break;
						}
					}
					_best_sequence.push_back(job);
				}*/


				std::cout << "\nElapsed time (s): " << elapsed_time_IP.count();

				std::cout << "\nObjective value = " << _best_value;
				std::cout << "\nSequence: ";
				for (auto&& job : _best_sequence)
					std::cout << job + 1 << " ";
			}
		}

	}


	void CPLEXIPAlt::clear_cplex()
	{
		int status = 0;
		char error_text[CPXMESSAGEBUFSIZE];

		// Free the problem
		status = CPXfreeprob(env, &problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::clear_cplex(). \nCouldn't free problem. \nReason: " + std::string(error_text));
		}

		// Close the cplex environment
		status = CPXcloseCPLEX(&env);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function CPLEXIP::clear_cplex(). \nCouldn't close cplex environment. \nReason: " + std::string(error_text));
		}
	}


}