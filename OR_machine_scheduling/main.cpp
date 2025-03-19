#include "algorithms.h"
#include "cxxopts.h"
#include <iostream>
#include <memory>



int main(int argc, char* argv[])
{
	try
	{
		cxxopts::Options options("OR -- Machine Scheduling",
			"This program implements several algorithms for the single machine scheduling problem to minimize the total tardiness of the jobs.");

		options.add_options()
			("algorithm", "The choice of algorithm. Possibilities:"
				"\n\t* \"CE\": complete enumeration"
				"\n\t* \"BB\": a dedicated branch-and-bound approach"
				"\n\t* \"CPLEX1\": an integer programming model solved with CPLEX (with variables x[j][k] = 1 if job j is at position k in the sequence, 0 otherwise)"
				"\n\t* \"CPLEX2\": an alternative integer programming model solved with CPLEX (with variables z[i][j] = 1 if job i comes before job j in the sequence, 0 otherwise)"
				"\n\t* \"IP1\": an integer programming model solved with SCIP (with variables x[j][k] = 1 if job j is at position k in the sequence, 0 otherwise)"
				"\n\t* \"IP2\": an alternative integer programming model solved with SCIP (with variables z[i][j] = 1 if job i comes before job j in the sequence, 0 otherwise)"
				, cxxopts::value<std::string>())
			("data", "Name of the file containing the problem data", cxxopts::value<std::string>())
			("verbose", "Explain the various steps of the algorithm", cxxopts::value<bool>())
			("help", "Help on how to use the application");



		auto result = options.parse(argc, argv);

		if (argc <= 1 || result.count("help"))
		{
			std::cout << options.help() << "\n\n\n\n\n";
			return EXIT_SUCCESS;
		}

		std::string algorithm;
		if (result.count("algorithm"))
			algorithm = result["algorithm"].as<std::string>();

		std::string datafile;
		if (result.count("data"))
			datafile = result["data"].as<std::string>();

		bool verbose = false;
		if (result.count("verbose"))
			verbose = result["verbose"].as<bool>();




		// create the algorithm and run it
		std::unique_ptr<MS::Algorithm> problem = MS::AlgorithmFactory::create(algorithm);
		problem->read_data(datafile);
		problem->run(verbose);



		std::cout << "\n\n\n\n\n";
		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		std::cout << "\n\n" << e.what() << "\n\n\n\n\n";
		return EXIT_FAILURE;
	}
}