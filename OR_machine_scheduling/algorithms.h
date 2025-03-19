#pragma once
#ifndef ALGORITHMS_MS_H
#define ALGORITHMS_MS_H


#include <exception>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include "ilcplex/cplex.h"
#include "ortools/linear_solver/linear_solver.h"


namespace MS // machine scheduling
{
	class Output
	{
		bool _on = true;

	public:
		void set_on(bool on) { _on = on; }
		friend Output& operator<<(Output& output, const std::string& msg);
		friend Output& operator<<(Output& output, const char* msg);
		friend Output& operator<<(Output& output, long unsigned int value);
		friend Output& operator<<(Output& output, size_t value);
		friend Output& operator<<(Output& output, unsigned int value);
		friend Output& operator<<(Output& output, int value);
		friend Output& operator<<(Output& output, int64_t value);
		friend Output& operator<<(Output& output, float value);
		friend Output& operator<<(Output& output, double value);
	};

	///////////////////////////////////////////////////////////////////////////

	struct Job
	{
		int duration;
		int due_date;
	};

	///////////////////////////////////////////////////////////////////////////

	// Base class
	class Algorithm
	{
	protected:
		Output _output;

		std::vector<Job> _jobs;

		int64_t _best_value = std::numeric_limits<int64_t>::max();
		std::vector<int> _best_sequence; // [i] == j  means position i is taken by job j

	public:
		virtual ~Algorithm() {}

		void read_data(const std::string& filename);
		void generate_dataset(size_t nb_jobs);

		virtual void run(bool verbose) = 0;
	};

	///////////////////////////////////////////////////////////////////////////

	class CompleteEnumeration : public Algorithm
	{
		void complete_enumeration_imp(std::vector<int>& sequence);

	public:
		void run(bool verbose) override;
	};

	///////////////////////////////////////////////////////////////////////////

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
		void run(bool verbose) override;
	};

	///////////////////////////////////////////////////////////////////////////

	// with x_jk = 1 if job j in position k, and C_k = completion time of job k
	class CPLEXIP : public Algorithm
	{
		CPXENVptr env = nullptr;
		CPXLPptr problem = nullptr;

		void initialize_cplex();
		void build_problem();
		void solve_problem();
		void clear_cplex();

		bool _output_screen = false;
		double _max_computation_time = 600; // seconds

	public:
		void run(bool verbose) override;
		void set_max_time(double time) { _max_computation_time = time; }
	};

	///////////////////////////////////////////////////////////////////////////

	// with z_ij = 1 if job i before job j
	class CPLEXIPAlt : public Algorithm
	{
		CPXENVptr env = nullptr;
		CPXLPptr problem = nullptr;

		void initialize_cplex();
		void build_problem();
		void solve_problem();
		void clear_cplex();

		bool _output_screen = false;
		double _max_computation_time = 600; // seconds

	public:
		void run(bool verbose) override;
		void set_max_time(double time) { _max_computation_time = time; }
	};


	///////////////////////////////////////////////////////////////////////////

	// with x_jk = 1 if job j in position k, and C_k = completion time of job k
	class ORToolsIP : public Algorithm
	{
		std::unique_ptr<operations_research::MPSolver> _solver; // OR Tools solver

		/*!
		 *	@brief Which solver is used?
		 *
		 *	solver_id is case insensitive, and the following names are supported:
		 *  - SCIP_MIXED_INTEGER_PROGRAMMING or SCIP
		 *  - CBC_MIXED_INTEGER_PROGRAMMING or CBC
		 *  - CPLEX_MIXED_INTEGER_PROGRAMMING or CPLEX or CPLEX_MIP			(license needed)
		 *  - GUROBI_MIXED_INTEGER_PROGRAMMING or GUROBI or GUROBI_MIP	    (license needed)
		 *  - XPRESS_MIXED_INTEGER_PROGRAMMING or XPRESS or XPRESS_MIP		(license needed)
		 */
		std::string _solver_type = "SCIP"; 

		void build_problem();
		void solve_problem();
		
		bool _output_screen = false;
		double _max_computation_time = 600; // seconds

	public:
		void run(bool verbose) override;
		void set_max_time(double time) { _max_computation_time = time; }
	};

	///////////////////////////////////////////////////////////////////////////

	// with z_ij = 1 if job i before job j
	class ORToolsIPAlt : public Algorithm
	{
		std::unique_ptr<operations_research::MPSolver> _solver; // OR Tools solver

		/*!
		 *	@brief Which solver is used?
		 *
		 *	solver_id is case insensitive, and the following names are supported:
		 *  - SCIP_MIXED_INTEGER_PROGRAMMING or SCIP
		 *  - CBC_MIXED_INTEGER_PROGRAMMING or CBC
		 *  - CPLEX_MIXED_INTEGER_PROGRAMMING or CPLEX or CPLEX_MIP			(license needed)
		 *  - GUROBI_MIXED_INTEGER_PROGRAMMING or GUROBI or GUROBI_MIP	    (license needed)
		 *  - XPRESS_MIXED_INTEGER_PROGRAMMING or XPRESS or XPRESS_MIP		(license needed)
		 */
		std::string _solver_type = "SCIP";

		void build_problem();
		void solve_problem();

		bool _output_screen = false;
		double _max_computation_time = 600; // seconds

	public:
		void run(bool verbose) override;
		void set_max_time(double time) { _max_computation_time = time; }
	};

	///////////////////////////////////////////////////////////////////////////

	class AlgorithmFactory
	{
	public:
		static std::unique_ptr<Algorithm> create(std::string& algorithm);
	};


} // namespace MS


#endif // !ALGORITHMS_MS_H