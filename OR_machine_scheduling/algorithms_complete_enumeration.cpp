#include "algorithms.h"
#include <chrono>
#include <stdexcept>


namespace MS
{
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

			_output << "\nSequence: ";
			for (auto&& job : sequence)
				_output << job + 1 << " ";
			_output << "Tardiness: " << tardiness;

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

	void CompleteEnumeration::run(bool verbose)
	{
		_output.set_on(true);
		_output << "\n\nStarting complete enumeration ...\n";
		_output.set_on(verbose);

		std::vector<int> sequence;
		std::vector<int> remaining_jobs;

		sequence.reserve(_jobs.size());

		auto start_time = std::chrono::system_clock::now();
		complete_enumeration_imp(sequence);
		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;

		_output.set_on(true);
		_output << "\n\nElapsed time (s): " << elapsed_time.count();
		_output << "\nBest sequence: ";
		for (auto&& job : _best_sequence)
			_output << job + 1 << " ";
		_output << "\nTardiness: " << _best_value;
	}
}