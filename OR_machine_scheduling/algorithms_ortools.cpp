#include "algorithms.h"
#include <stdexcept>
#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <queue>


namespace MS
{
	void ORToolsIP::build_problem()
	{
		// create the solver (_solver_type == SCIP or CPLEX or ...)
		_solver.reset(operations_research::MPSolver::CreateSolver(_solver_type));



		// add variables
		const size_t nbjobs = _jobs.size();
		int nb_variables = -1;
		const double infinity = _solver->infinity();


		// variables x_jk
		for (size_t j = 0; j < nbjobs; ++j)
		{
			for (size_t k = 0; k < nbjobs; ++k)
			{
				std::string varname = "x_" + std::to_string(j + 1) + "_" + std::to_string(k + 1);
				operations_research::MPVariable* var = _solver->MakeBoolVar(varname);
			}
		}

		// variables C_k
		for (size_t k = 0; k < nbjobs; ++k)
		{
			std::string varname = "C_" + std::to_string(k + 1);
			operations_research::MPVariable* var = _solver->MakeNumVar(0.0, infinity, varname);
		}

		// variables T_j
		for (size_t j = 0; j < nbjobs; ++j)
		{
			std::string varname = "T_" + std::to_string(j + 1);
			operations_research::MPVariable* var = _solver->MakeNumVar(0.0, infinity, varname);
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



		// set objective function
		operations_research::MPObjective* objective = _solver->MutableObjective();
		objective->SetMinimization();
		for (auto j = 0; j < _jobs.size(); ++j)
		{
			size_t index_var = index_T_j(j);
			operations_research::MPVariable* var = _solver->variable(index_var);
			objective->SetCoefficient(var, 1);
		}



		// add constraints
		int nb_constraints = -1;

		// 1: every job in exactly one position in the sequence
		for (int j = 0; j < nbjobs; ++j)
		{
			++nb_constraints;

			std::string conname = "c1_" + std::to_string(j + 1);
			operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(1.0, 1.0, conname);

			// x_jk
			for (int k = 0; k < nbjobs; ++k)
			{
				size_t index_var = index_x_jk(j, k);
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, 1);
			}
		}

		// 2: every position in the sequence has exactly one job
		for (int k = 0; k < nbjobs; ++k)
		{
			++nb_constraints;

			std::string conname = "c2_" + std::to_string(k + 1);
			operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(1.0, 1.0, conname);

			// x_jk
			for (int j = 0; j < nbjobs; ++j)
			{
				size_t index_var = index_x_jk(j, k);
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, 1);
			}
		}

		// 3 & 4: completion time job in position k
		for (int k = 0; k < nbjobs; ++k)
		{
			++nb_constraints;

			std::string conname = "c3and4_" + std::to_string(k + 1);
			operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(0.0, infinity, conname);

			// x_jk
			for (int j = 0; j < nbjobs; ++j)
			{
				size_t index_var = index_x_jk(j, k);
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, -_jobs[j].duration);
			}

			// C_k
			{
				size_t index_var = index_C_k(k);
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, 1);
			}

			// C_k-1
			if (k >= 1)
			{
				size_t index_var = index_C_k(k-1);
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, -1);
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

				std::string conname = "c5_" + std::to_string(j+1) + "_" + std::to_string(k + 1);
				operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(-_jobs[j].due_date - BigM, infinity, conname);

				// x_jk
				{
					size_t index_var = index_x_jk(j, k);
					operations_research::MPVariable* var = _solver->variable(index_var);
					constraint->SetCoefficient(var, -BigM);
				}

				// C_k
				{
					size_t index_var = index_C_k(k);
					operations_research::MPVariable* var = _solver->variable(index_var);
					constraint->SetCoefficient(var, -1);
				}

				// T_j
				{
					size_t index_var = index_T_j(j);
					operations_research::MPVariable* var = _solver->variable(index_var);
					constraint->SetCoefficient(var, 1);
				}
			}
		}





		// write to file
		//_solver->Write("ORToolsIP.lp"); // NOT YET SUPPORTED

		{
			std::ofstream mfile("ORTools_IP.lp");
			mfile << "Obj\t";
			for (auto& var : _solver->variables())
			{
				if (_solver->Objective().GetCoefficient(var) > 0.00001 || _solver->Objective().GetCoefficient(var) < -0.00001)
					mfile << _solver->Objective().GetCoefficient(var) << " " << var->name() << " + ";
			}

			for (auto& con : _solver->constraints())
			{
				mfile << "\n\n" << con->name() << "\t" << con->lb() << " <=  ";
				for (auto& var : _solver->variables())
				{
					if (con->GetCoefficient(var) > 0.00001 || con->GetCoefficient(var) < -0.00001)
						mfile << con->GetCoefficient(var) << " " << var->name() << " + ";
				}
				mfile << "  <= " << con->ub();
			}
		}
	}

	void ORToolsIP::solve_problem()
	{
		std::cout << "\nUsing an IP model with x_jk = 1 if job j is at position k in the sequence, 0 otherwise"
			<< "\nUsing ORTools with SCIP to solve the model ...\n\n";

		// Output to screen
		if (_output_screen)
			_solver->EnableOutput();
		else
			_solver->SuppressOutput();

		// Set time limit (milliseconds) 
		int64_t time_limit = static_cast<int64_t>(_max_computation_time * 1000);
		_solver->set_time_limit(time_limit);

		// Solve the problem
		auto start_time = std::chrono::system_clock::now();
		const operations_research::MPSolver::ResultStatus result_status = _solver->Solve();
		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time_IP = std::chrono::system_clock::now() - start_time;

		std::cout << "\nResult solve = " << result_status;

		// If optimal or feasible (e.g. time limit reached)
		if (result_status == operations_research::MPSolver::OPTIMAL || result_status == operations_research::MPSolver::FEASIBLE)
		{
			double objval = _solver->Objective().Value();
			_best_value = objval + 0.0001;

			_best_sequence.clear();
			_best_sequence.reserve(_jobs.size());

			for (int k = 0; k < _jobs.size(); ++k) {
				int job = 0;
				for (int j = 0; j < _jobs.size(); ++j) {

					int index_var = j * _jobs.size() + k;
					operations_research::MPVariable* var = _solver->variable(index_var);
					double solvalue = var->solution_value();

					if (solvalue > 0.99) {
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

	void ORToolsIP::run(bool verbose)
	{
		_output_screen = verbose;

		build_problem();
		solve_problem();
	}

	///////////////////////////////////////////////////////////////////////////

	void ORToolsIPAlt::build_problem()
	{
		// create the solver (_solver_type == SCIP or CPLEX or ...)
		_solver.reset(operations_research::MPSolver::CreateSolver(_solver_type));



		// add variables
		const size_t nbjobs = _jobs.size();
		int nb_variables = -1;
		const double infinity = _solver->infinity();


		// variables z_ij
		for (size_t i = 0; i < nbjobs; ++i)
		{
			for (size_t j = 0; j < nbjobs; ++j)
			{
				std::string varname = "z_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
				operations_research::MPVariable* var = _solver->MakeBoolVar(varname);
			}
		}

		// variables T_j
		for (size_t j = 0; j < nbjobs; ++j)
		{
			std::string varname = "T_" + std::to_string(j + 1);
			operations_research::MPVariable* var = _solver->MakeNumVar(0.0, infinity, varname);
		}



		// lambdas to access variables
		auto index_z_ij = [nbjobs](int i, int j) -> int {
			return i * nbjobs + j;
			};

		auto index_T_j = [nbjobs](int j) -> int {
			return nbjobs * nbjobs + j;
			};



		// set objective function
		operations_research::MPObjective* objective = _solver->MutableObjective();
		objective->SetMinimization();
		for (auto j = 0; j < _jobs.size(); ++j)
		{
			size_t index_var = index_T_j(j);
			operations_research::MPVariable* var = _solver->variable(index_var);
			objective->SetCoefficient(var, 1);
		}



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

					std::string conname = "c1_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
					operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(1.0, 1.0, conname);

					// z_ij
					{
						size_t index_var = index_z_ij(i,j);
						operations_research::MPVariable* var = _solver->variable(index_var);
						constraint->SetCoefficient(var, 1);
					}

					// z_ji
					{
						size_t index_var = index_z_ij(j,i);
						operations_research::MPVariable* var = _solver->variable(index_var);
						constraint->SetCoefficient(var, 1);
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

						std::string conname = "c2_" + std::to_string(i + 1) + "_" + std::to_string(j + 1) + "_" + std::to_string(k + 1);
						operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(-infinity, 1.0, conname);

						// z_ij
						{
							size_t index_var = index_z_ij(i, j);
							operations_research::MPVariable* var = _solver->variable(index_var);
							constraint->SetCoefficient(var, 1);
						}

						// z_jk
						{
							size_t index_var = index_z_ij(j, k);
							operations_research::MPVariable* var = _solver->variable(index_var);
							constraint->SetCoefficient(var, 1);
						}

						// z_ik
						{
							size_t index_var = index_z_ij(i, k);
							operations_research::MPVariable* var = _solver->variable(index_var);
							constraint->SetCoefficient(var, -1);
						}
					}
				}
			}
		}

		// 3: T_j - sum(i) p_i z_ij >= p_j - d_j   forall j
		for (int j = 0; j < nbjobs; ++j)
		{
			++nb_constraints;

			std::string conname = "c3_" + std::to_string(j + 1);
			operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(_jobs[j].duration - _jobs[j].due_date, infinity, conname);

			// - sum(i) p_i z_ij
			for(int i = 0; i < nbjobs; ++i)
			{
				size_t index_var = index_z_ij(i, j);
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, -_jobs[i].duration);
			}

			// T_j
			{
				size_t index_var = index_T_j(j);
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, 1);
			}
		}

		// write to file
		//_solver->Write("ORToolsIP.lp"); // NOT YET SUPPORTED

		{
			std::ofstream mfile("ORTools_IPAlt.lp");
			mfile << "Obj\t";
			for (auto& var : _solver->variables())
			{
				if (_solver->Objective().GetCoefficient(var) > 0.00001 || _solver->Objective().GetCoefficient(var) < -0.00001)
					mfile << _solver->Objective().GetCoefficient(var) << " " << var->name() << " + ";
			}

			for (auto& con : _solver->constraints())
			{
				mfile << "\n\n" << con->name() << "\t" << con->lb() << " <=  ";
				for (auto& var : _solver->variables())
				{
					if (con->GetCoefficient(var) > 0.00001 || con->GetCoefficient(var) < -0.00001)
						mfile << con->GetCoefficient(var) << " " << var->name() << " + ";
				}
				mfile << "  <= " << con->ub();
			}
		}
	}

	void ORToolsIPAlt::solve_problem()
	{
		std::cout << "\nUsing an IP model with z_ij = 1 if job i is processed before job j, 0 otherwise"
			<< "\nUsing ORTools with SCIP to solve the model ...\n\n";

		// Output to screen
		if (_output_screen)
			_solver->EnableOutput();
		else
			_solver->SuppressOutput();

		// Set time limit (milliseconds) 
		int64_t time_limit = static_cast<int64_t>(_max_computation_time * 1000);
		_solver->set_time_limit(time_limit);

		// Solve the problem
		auto start_time = std::chrono::system_clock::now();
		const operations_research::MPSolver::ResultStatus result_status = _solver->Solve();
		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time_IP = std::chrono::system_clock::now() - start_time;

		std::cout << "\nResult solve = " << result_status;

		// If optimal or feasible (e.g. time limit reached)
		if (result_status == operations_research::MPSolver::OPTIMAL || result_status == operations_research::MPSolver::FEASIBLE)
		{
			double objval = _solver->Objective().Value();
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

						int index_var = i * _jobs.size() + j;
						operations_research::MPVariable* var = _solver->variable(index_var);
						double solvalue = var->solution_value();

						if (i != j && solvalue > 0.99) { // i precedes j
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


			std::cout << "\nElapsed time (s): " << elapsed_time_IP.count();

			std::cout << "\nObjective value = " << _best_value;
			std::cout << "\nSequence: ";
			for (auto&& job : _best_sequence)
				std::cout << job + 1 << " ";
		}
	}

	void ORToolsIPAlt::run(bool verbose)
	{
		_output_screen = verbose;

		build_problem();
		solve_problem();
	}

}