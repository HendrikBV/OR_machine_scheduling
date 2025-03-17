#include "algorithms.h"
#include <fstream>
#include <random>
#include <stdexcept>
#include <iostream>



namespace MS
{

	Output& operator<<(Output& output, const std::string& msg) 
	{
		if (output._on) {
			std::cout << msg;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, const char* msg)
	{
		if (output._on) {
			std::cout << msg;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, long unsigned int value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, size_t value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, unsigned int value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, int value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, int64_t value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, float value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, double value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}


	///////////////////////////////////////////////////////////////////////////


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


	///////////////////////////////////////////////////////////////////////////


	std::unique_ptr<Algorithm> AlgorithmFactory::create(std::string& algorithm)
	{
		std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(),
			[](unsigned char c) { return std::tolower(c); });


		if (algorithm == "ce")
			return std::make_unique<CompleteEnumeration>();
		else if (algorithm == "bb")
			return std::make_unique<BranchAndBound>();
		else if (algorithm == "cplex1")
			return std::make_unique<CPLEXIP>();
		else if (algorithm == "cplex2")
			return std::make_unique<CPLEXIPAlt>();
		else if (algorithm == "ip1")
			return std::make_unique<ORToolsIP>();
		else if (algorithm == "ip2")
			return std::make_unique<ORToolsIPAlt>();
		else
			throw std::invalid_argument("No algorithm " + algorithm + " exists");
	}
}