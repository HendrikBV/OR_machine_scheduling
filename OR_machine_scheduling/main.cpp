#include <exception>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <list>
#include <chrono>
#include <random>
#include "ilcplex/cplex.h"



struct Job
{
	int duration;
	int due_date;
};


class Algorithm
{
protected:
	std::vector<Job> _jobs;

	int64_t _best_value = std::numeric_limits<int64_t>::max();
	std::vector<int> _best_sequence; // [i] == j  means position i is taken by job j

public:
	virtual ~Algorithm() {}

	void read_data(const std::string& filename);
	void generate_dataset(size_t nb_jobs);

	virtual void run() = 0;
	virtual void run_explanation() = 0;
};


class CompleteEnumeration : public Algorithm
{
	void complete_enumeration_imp_exp(std::vector<int>& sequence);
	void complete_enumeration_imp(std::vector<int>& sequence);

public:
	void run() override;
	void run_explanation() override;
};


class BranchAndBound : public Algorithm
{
	struct Node
	{
		int number = 0;
		std::vector<int> sequence; // reversed sequence
		int64_t lowerbound = 0;

		bool operator==(const Node& other) const { if (sequence == other.sequence && lowerbound == other.lowerbound) return true; return false; }
	};
	std::vector<Node> _nodes;
	size_t _nb_nodes = 0;
	size_t _nb_nodes_pruned = 0;

public: 
	void run() override;
	void run_explanation() override;
};


class IPModel : public Algorithm
{
	CPXENVptr env;
	CPXLPptr problem;

	void initialize_cplex();
	void build_problem();
	void solve_problem();
	void clear_cplex();

	bool _output = false;
	double _max_computation_time = 120; // seconds

public:
	void run() override;
	void run_explanation() override;
};



void Algorithm::read_data(const std::string& filename)
{
	std::ifstream file;
	file.open(filename);
	if (!file.is_open())
	{
		throw std::runtime_error("Couldn't open file " + filename);
	}

	_jobs.clear();
	size_t nb_jobs;
	std::string text;
	file >> text >> nb_jobs >> text >> text;

	for (size_t i = 0; i < nb_jobs; ++i)
	{
		int dur, due;
		file >> dur >> due;
		_jobs.push_back(Job());
		_jobs.back().duration = dur;
		_jobs.back().due_date = due;
	}
}


void Algorithm::generate_dataset(size_t nb_jobs)
{
	std::random_device randdev;
	std::seed_seq seedseq{ randdev(),randdev(), randdev(), randdev(), randdev(), randdev(), randdev() };
	std::mt19937_64 generator{ seedseq };

	std::uniform_int_distribution<> dist_duration(5, 10);
	std::uniform_int_distribution<> dist_duedate(10, nb_jobs * 7);

	std::ofstream file("dataset.txt");
	if (!file.is_open())
		throw std::runtime_error("Couldn't open output file");

	file << "nb_jobs\t" << nb_jobs << "\n\nduration\tdue_date";
	for (auto i = 0; i < nb_jobs; ++i)
	{
		file << "\n" << dist_duration(generator) << "\t" << dist_duedate(generator);
	}
}



void CompleteEnumeration::complete_enumeration_imp_exp(std::vector<int>& sequence)
{
	if (sequence.size() == _jobs.size())
	{
		int64_t tardiness = 0;
		int64_t total_duration = 0;
		for (auto&& job: sequence)
		{
			total_duration += _jobs[job].duration;
			tardiness += std::max(total_duration - _jobs[job].due_date, 0ll);
		}

		std::cout << "\nSequence: ";
		for (auto&& job : sequence)
			std::cout << job+1 << " ";
		std::cout << "Tardiness: " << tardiness;

		if (tardiness < _best_value)
		{
			_best_value = tardiness;
			_best_sequence = sequence;
		}

		return;
	}

	// loop over remaining _jobs
	for (size_t j = 0; j < _jobs.size(); ++j)
	{
		bool already_added = false;
		for (auto&& i : sequence)
		{
			if (i == j)
			{
				already_added = true;
				break;
			}
		}

		if (!already_added)
		{
			sequence.push_back(j);
			complete_enumeration_imp_exp(sequence);
			sequence.pop_back();
		}
	}
}


void CompleteEnumeration::complete_enumeration_imp(std::vector<int>& sequence)
{
	if (sequence.size() == _jobs.size())
	{
		int64_t tardiness = 0;
		int64_t total_duration = 0;
		for (auto&& job : sequence)
		{
			total_duration += _jobs[job].duration;
			tardiness += std::max(total_duration - _jobs[job].due_date, 0ll);
		}

		if (tardiness < _best_value)
		{
			_best_value = tardiness;
			_best_sequence = sequence;
		}

		return;
	}

	// loop over remaining _jobs
	for (size_t j = 0; j < _jobs.size(); ++j)
	{
		bool already_added = false;
		for (auto&& i : sequence)
		{
			if (i == j)
			{
				already_added = true;
				break;
			}
		}

		if (!already_added)
		{
			sequence.push_back(j);
			complete_enumeration_imp(sequence);
			sequence.pop_back();
		}
	}
}


void CompleteEnumeration::run()
{
	std::cout << "\n\nStarting complete enumeration ...\n";

	std::vector<int> sequence;
	std::vector<int> remaining_jobs;

	sequence.reserve(_jobs.size());

	auto start_time = std::chrono::system_clock::now();
	complete_enumeration_imp(sequence);
	std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;

	std::cout << "\n\nElapsed time (s): " << elapsed_time.count();
	std::cout << "\nBest sequence: ";
	for (auto&& job : _best_sequence)
		std::cout << job + 1 << " ";
	std::cout << "\nTardiness: " << _best_value;
}


void CompleteEnumeration::run_explanation()
{
	std::cout << "\n\nStarting complete enumeration ...\n";

	std::vector<int> sequence;
	std::vector<int> remaining_jobs;

	sequence.reserve(_jobs.size());

	auto start_time = std::chrono::system_clock::now();
	complete_enumeration_imp_exp(sequence);
	std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;

	std::cout << "\n\nElapsed time (s): " << elapsed_time.count();
	std::cout << "\nBest sequence: ";
	for (auto&& job : _best_sequence)
		std::cout << job+1 << " ";
	std::cout << "\nTardiness: " << _best_value;
}



void BranchAndBound::run()
{
	std::cout << "\nStarting branch-and-bound ...\n";

	auto start_time = std::chrono::system_clock::now();

	// heuristic
	std::cout << "\nUse a due-date based heuristic for an initial upper bound";
	_best_sequence.reserve(_jobs.size());
	{
		int totalduration = 0;
		int totaltardiness = 0;

		for (auto k = 0; k < _jobs.size(); ++k)
		{
			int smallest_due_date = std::numeric_limits<int>::max();
			int best_job = 0;

			for (auto j = 0; j < _jobs.size(); ++j)
			{
				bool alread_assigned = false;
				for (auto&& l : _best_sequence)
				{
					if (l == j)
					{
						alread_assigned = true;
						break;
					}
				}

				if (!alread_assigned)
				{
					if (_jobs[j].due_date < smallest_due_date)
					{
						smallest_due_date = _jobs[j].due_date;
						best_job = j;
					}
				}
			}

			_best_sequence.push_back(best_job);
			totalduration += _jobs[best_job].duration;
			totaltardiness += std::max(totalduration - _jobs[best_job].due_date, 0);
		}

		_best_value = totaltardiness;
		std::cout << "\nInitial upper bound: " << totaltardiness;
	}

	// initial nodes: last position in sequence
	{
		int64_t total_duration = 0;
		for (auto&& j : _jobs)
			total_duration += j.duration;

		for (size_t j = 0; j < _jobs.size(); ++j)
		{
			++_nb_nodes;
			_nodes.push_back(Node());
			_nodes.back().number = _nb_nodes;
			_nodes.back().lowerbound = std::max(total_duration - _jobs[j].due_date, 0ll);
			_nodes.back().sequence.push_back(j);
		}
	}

	// explore until all dones done
	while (true)
	{
		// eliminate nodes with LB >= UB
		for (int64_t index_node = _nodes.size() - 1; index_node >= 0; --index_node)
		{
			if (_nodes[index_node].lowerbound >= _best_value)
			{
				std::swap(_nodes[index_node], _nodes.back());  // Swap with last element
				_nodes.pop_back();

				++_nb_nodes_pruned;
			}
		}

		// if no nodes left, stop
		if (_nodes.empty())
			break;

		// otherwise take best node
		Node node;
		{
			int64_t best_bound = std::numeric_limits<int64_t>::max();
			int64_t index_best_node = _nodes.size() - 1;

			for (auto index_node = 0; index_node < _nodes.size(); ++index_node)
			{
				if (_nodes[index_node].lowerbound < best_bound)
				{
					index_best_node = index_node;
					node = _nodes[index_node];
					best_bound = node.lowerbound;
				}
			}

			// delete chosen node
			std::swap(_nodes[index_best_node], _nodes.back());  // Swap with last element
			_nodes.pop_back();
		}


		// finish sequence if only one job remaining
		if (node.sequence.size() == _jobs.size() - 1)
		{
			for (size_t j = 0; j < _jobs.size(); ++j)
			{
				bool already_added = false;
				for (auto&& i : node.sequence)
				{
					if (i == j)
					{
						already_added = true;
						break;
					}
				}

				if (!already_added)
				{
					node.lowerbound += std::max(_jobs[j].duration - _jobs[j].due_date, 0);
					node.sequence.push_back(j);

					break;
				}
			}

			if (node.lowerbound < _best_value)
			{
				_best_value = node.lowerbound;
				_best_sequence = node.sequence;
			}
		}




		// branch one level further
		else
		{
			// calculate length of unassigned jobs
			int64_t total_duration = 0;
			for (size_t j = 0; j < _jobs.size(); ++j)
			{
				bool already_added = false;
				for (auto&& i : node.sequence)
				{
					if (i == j)
					{
						already_added = true;
						break;
					}
				}

				if (!already_added)
					total_duration += _jobs[j].duration;
			}


			// add all remaining possibilities to the sequence
			for (size_t j = 0; j < _jobs.size(); ++j)
			{
				bool already_added = false;
				for (auto&& i : node.sequence)
				{
					if (i == j)
					{
						already_added = true;
						break;
					}
				}

				if (!already_added)
				{
					++_nb_nodes;
					_nodes.push_back(Node());
					_nodes.back().number = _nb_nodes;
					_nodes.back().sequence = node.sequence;
					_nodes.back().sequence.push_back(j);
					_nodes.back().lowerbound = node.lowerbound + std::max(total_duration - _jobs[j].due_date, 0ll);
				}
			}
		}
	}



	std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;

	std::cout << "\n\nElapsed time (s): " << elapsed_time.count();
	std::cout << "\nNodes evaluated: " << _nb_nodes;
	std::cout << "\nNodes pruned: " << _nb_nodes_pruned;
	std::cout << "\nBest sequence: ";
	for (int64_t j = _jobs.size() - 1; j >= 0; --j)
		std::cout << _best_sequence[j] + 1 << " "; // reverse the sequence
	std::cout << "\nTardiness: " << _best_value;
}


void BranchAndBound::run_explanation()
{
	std::cout << "\nStarting branch-and-bound ...\n";

	auto start_time = std::chrono::system_clock::now();

	// heuristic
	std::cout << "\nUse a due-date based heuristic for an initial upper bound";
	_best_sequence.reserve(_jobs.size());
	{
		int totalduration = 0;
		int totaltardiness = 0;

		for (auto k = 0; k < _jobs.size(); ++k)
		{
			int smallest_due_date = std::numeric_limits<int>::max();
			int best_job = 0;

			for (auto j = 0; j < _jobs.size(); ++j)
			{
				bool alread_assigned = false;
				for (auto&& l : _best_sequence)
				{
					if (l == j)
					{
						alread_assigned = true;
						break;
					}
				}

				if (!alread_assigned)
				{
					if (_jobs[j].due_date < smallest_due_date)
					{
						smallest_due_date = _jobs[j].due_date;
						best_job = j;
					}
				}
			}

			_best_sequence.push_back(best_job);
			totalduration += _jobs[best_job].duration;
			totaltardiness += std::max(totalduration - _jobs[best_job].due_date, 0);
		}

		_best_value = totaltardiness;
		std::cout << "\nInitial upper bound: " << totaltardiness;
		std::cout << "\nSequence: ";
		for (auto&& job : _best_sequence)
			std::cout << job + 1 << " ";
	}

	// initial nodes: last position in sequence
	std::cout << "\n\nCreating initial nodes";
	{
		int64_t total_duration = 0;
		for (auto&& j : _jobs)
			total_duration += j.duration;

		for (size_t j = 0; j < _jobs.size(); ++j)
		{
			++_nb_nodes;
			_nodes.push_back(Node());
			_nodes.back().number = _nb_nodes;
			_nodes.back().lowerbound = std::max(total_duration - _jobs[j].due_date, 0ll);
			_nodes.back().sequence.push_back(j);

			std::cout << "\nNode " << _nodes.back().number << " with x(" << j + 1 << "," << _jobs.size() << ") = 1, LB = " << _nodes.back().lowerbound;
		}
	}

	// explore until all dones done
	while (true)
	{
		// eliminate nodes with LB >= UB
		for (int64_t index_node = _nodes.size() - 1; index_node >= 0; --index_node)
		{
			if (_nodes[index_node].lowerbound >= _best_value)
			{
				std::cout << "\n\nNode " << _nodes[index_node].number << " with partial sequence ";
				for (int64_t k = _nodes[index_node].sequence.size() - 1; k >= 0; --k)
					std::cout << _nodes[index_node].sequence[k] + 1 << " ";
				std::cout << "has LB = " << _nodes[index_node].lowerbound << " >= UB = "
					<< _best_value << ", so the node is dominated;";

				std::swap(_nodes[index_node], _nodes.back());  // Swap with last element
				_nodes.pop_back();

				++_nb_nodes_pruned;
			}
		}

		// if no nodes left, stop
		if (_nodes.empty())
			break;

		// otherwise take best node
		Node node;
		{
			int64_t best_bound = std::numeric_limits<int64_t>::max();
			int64_t index_best_node = _nodes.size() - 1;

			for (auto index_node = 0; index_node < _nodes.size(); ++index_node)
			{
				if (_nodes[index_node].lowerbound < best_bound)
				{
					index_best_node = index_node;
					node = _nodes[index_node];
					best_bound = node.lowerbound;
				}
			}

			// delete chosen node
			std::swap(_nodes[index_best_node], _nodes.back());  // Swap with last element
			_nodes.pop_back();
		}


		std::cout << "\n\nBest remaining node is Node " << node.number << " with partial sequence ";
		for (int64_t k = node.sequence.size() - 1; k >= 0; --k)
			std::cout << node.sequence[k] + 1 << " ";
		std::cout << " and LB = " << node.lowerbound;


		// finish sequence if only one job remaining
		if (node.sequence.size() == _jobs.size() - 1)
		{
			for (size_t j = 0; j < _jobs.size(); ++j)
			{
				bool already_added = false;
				for (auto&& i : node.sequence)
				{
					if (i == j)
					{
						already_added = true;
						break;
					}
				}

				if (!already_added)
				{
					node.lowerbound += std::max(_jobs[j].duration - _jobs[j].due_date, 0);
					node.sequence.push_back(j);

					std::cout << "\nComplete sequence ( ";
					for (int64_t k = node.sequence.size() - 1; k >= 0; --k)
						std::cout << node.sequence[k] + 1 << " ";
					std::cout << ") found with LB = " << node.lowerbound;

					break;
				}
			}

			if (node.lowerbound < _best_value)
			{
				_best_value = node.lowerbound;
				_best_sequence = node.sequence;

				std::cout << "\nNew best solution found with sequence ";
				for (int64_t k = _best_sequence.size() - 1; k >= 0; --k)
					std::cout << _best_sequence[k] + 1 << " ";
				std::cout << ", update UB = " << _best_value;
			}
		}




		// branch one level further
		else
		{ 
			std::cout << "\nBranch one level further";

			// calculate length of unassigned jobs
			int64_t total_duration = 0;
			for (size_t j = 0; j < _jobs.size(); ++j)
			{
				bool already_added = false;
				for (auto&& i : node.sequence)
				{
					if (i == j)
					{
						already_added = true;
						break;
					}
				}

				if (!already_added)
					total_duration += _jobs[j].duration;
			}


			// add all remaining possibilities to the sequence
			for (size_t j = 0; j < _jobs.size(); ++j)
			{
				bool already_added = false;
				for (auto&& i : node.sequence)
				{
					if (i == j)
					{
						already_added = true;
						break;
					}
				}

				if (!already_added)
				{
					++_nb_nodes;
					_nodes.push_back(Node());
					_nodes.back().number = _nb_nodes;
					_nodes.back().sequence = node.sequence;
					_nodes.back().sequence.push_back(j);
					_nodes.back().lowerbound = node.lowerbound + std::max(total_duration - _jobs[j].due_date, 0ll);

					std::cout << "\nCreate a new node: Node " << _nodes.back().number << " with sequence ";
					for (int64_t k = _nodes.back().sequence.size() - 1; k >= 0; --k)
						std::cout << _nodes.back().sequence[k] + 1 << " ";
					std::cout << " and LB = " << _nodes.back().lowerbound;
				}
			}
		}
	}



	std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;

	std::cout << "\n\nElapsed time (s): " << elapsed_time.count();
	std::cout << "\nNodes evaluated: " << _nb_nodes;
	std::cout << "\nNodes pruned: " << _nb_nodes_pruned;
	std::cout << "\nBest sequence: ";
	for (int64_t j = _jobs.size() - 1; j >= 0; --j)
		std::cout << _best_sequence[j] + 1 << " "; // reverse the sequence
	std::cout << "\nTardiness: " << _best_value;
}



void IPModel::run()
{
	_output = false;

	initialize_cplex();
	build_problem();
	solve_problem();
	clear_cplex();
}


void IPModel::run_explanation()
{
	_output = true;

	initialize_cplex();
	build_problem();
	solve_problem();
	clear_cplex();
}


void IPModel::initialize_cplex()
{
	int status = 0;
	char error_text[CPXMESSAGEBUFSIZE];

	// open the CPLEX environment
	env = CPXopenCPLEX(&status);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::initialize_cplex(). \nCouldn't open CPLEX. \nReason: " + std::string(error_text));
	}

	// turn output to screen on/off
	if (_output)
		status = CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_ON);
	else
		status = CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_OFF);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::initialize_cplex(). \nCouldn't change param SCRIND. \nReason: " + std::string(error_text));
	}
}


void IPModel::build_problem()
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
		throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't create problem. \nReason: " + std::string(error_text));
	}

	// problem is minimization
	status = CPXchgobjsen(env, problem, CPX_MIN);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change objective to minimization. \nReason: " + std::string(error_text));
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
				throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
			}

			// change variable name
			std::string varname = "x_" + std::to_string(j + 1) + "_" + std::to_string(k + 1);
			status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
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
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
		}

		// change variable name
		std::string varname = "C_" + std::to_string(k + 1);
		status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
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
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't add variable. \nReason: " + std::string(error_text));
		}

		// change variable name
		std::string varname = "T_" + std::to_string(j + 1);
		status = CPXchgname(env, problem, 'c', nb_variables, varname.c_str());
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change variable name. \nReason: " + std::string(error_text));
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
			throw std::runtime_error("Error in function IPModel::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

		status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
		}

		// change name of constraint
		std::string conname = "c1_" + std::to_string(j + 1);
		status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
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
			throw std::runtime_error("Error in function IPModel::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

		status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
		}

		// change name of constraint
		std::string conname = "c2_" + std::to_string(k + 1);
		status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
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
			matind[nonzeroes] = index_C_k(k-1);
			matval[nonzeroes] = -1;
			++nonzeroes;
		}

		if (nonzeroes >= maxnonzeroes)
			throw std::runtime_error("Error in function IPModel::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

		status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
		}

		// change name of constraint
		std::string conname = "c3and4_" + std::to_string(k + 1);
		status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
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
				throw std::runtime_error("Error in function IPModel::build_problem(). Nonzeroes exceeds size of maxnonzeroes (matind and matval)");

			status = CPXaddrows(env, problem, 0, 1, nonzeroes, rhs, sense, matbeg, matind.get(), matval.get(), NULL, NULL);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't add constraint. \nReason: " + std::string(error_text));
			}

			// change name of constraint
			std::string conname = "c5_" + std::to_string(k + 1);
			status = CPXchgname(env, problem, 'r', nb_constraints, conname.c_str());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't change constraint name. \nReason: " + std::string(error_text));
			}
		}
	}



	// write to file
	status = CPXwriteprob(env, problem, "IPModel.lp", NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::build_problem(). \nCouldn't write problem to lp-file. \nReason: " + std::string(error_text));
	}
}


void IPModel::solve_problem()
{
	char error_text[CPXMESSAGEBUFSIZE];
	int status = 0;
	int solstat = 0;
	std::unique_ptr<double[]> solution_problem;
	double objval;

	/*status = CPXreadcopyprob(env, problem, "IPModel.lp", NULL); /// Dit werkt wel, maar probleem rechtstreeks geeft error
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::solve_problem(). \nCouldn't read problem from lp-file. \nReason: " + std::string(error_text));
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
	auto start_time = std::chrono::system_clock::now();

	status = CPXmipopt(env, problem);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
	}

	std::chrono::duration<double, std::ratio<1, 1>> elapsed_time_IP = std::chrono::system_clock::now() - start_time;


	// Get the solution
	status = CPXsolution(env, problem, &solstat, &objval, solution_problem.get(), NULL, NULL, NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
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


void IPModel::clear_cplex()
{
	int status = 0;
	char error_text[CPXMESSAGEBUFSIZE];

	// Free the problem
	status = CPXfreeprob(env, &problem);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::clear_cplex(). \nCouldn't free problem. \nReason: " + std::string(error_text));
	}

	// Close the cplex environment
	status = CPXcloseCPLEX(&env);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function IPModel::clear_cplex(). \nCouldn't close cplex environment. \nReason: " + std::string(error_text));
	}
}





class AlgorithmFactory
{
public:
	static std::unique_ptr<Algorithm> create(const std::string& algorithm)
	{
		if (algorithm == "CE" || algorithm == "complete enumeration")
			return std::make_unique<CompleteEnumeration>();
		else if (algorithm == "BB" || algorithm == "branch and bound")
			return std::make_unique<BranchAndBound>();
		else if (algorithm == "IP" || algorithm == "integer programming")
			return std::make_unique<IPModel>();
		else
			throw std::invalid_argument("No algorithm " + algorithm + " exists");
	}
};





int main()
{
	try
	{
		std::string instance = "dataset.txt";

		std::unique_ptr<Algorithm> IP = AlgorithmFactory::create("IP");
		IP->read_data(instance);
		IP->run();

		std::cout << "\n\n\n";

		std::unique_ptr<Algorithm> BB = AlgorithmFactory::create("BB");
		//BB->generate_dataset(20);
		BB->read_data(instance);
		BB->run();

		//std::unique_ptr<Algorithm> CE = AlgorithmFactory::create("CE");
		//CE->read_data(instance);
		//CE->run();


		std::cout << "\n\n\n\n\n";
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "\n\n" << e.what() << "\n\n\n\n\n";
		return 1;
	}
}