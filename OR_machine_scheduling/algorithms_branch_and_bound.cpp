#include "algorithms.h"
#include <chrono>
#include <stdexcept>

namespace MS
{
	void BranchAndBound::run(bool verbose)
	{
		_output.set_on(true);
		_output << "\nStarting branch-and-bound ...\n";
		_output.set_on(verbose);

		auto start_time = std::chrono::system_clock::now();

		// heuristic
		_output << "\nUse a due-date based heuristic for an initial upper bound";
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
			_output << "\nInitial upper bound: " << totaltardiness;
			_output << "\nSequence: ";
			for (auto&& job : _best_sequence)
				_output << job + 1 << " ";
		}

		// initial nodes: last position in sequence
		_output << "\n\nCreating initial nodes";
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

				_output << "\nNode " << _nodes.back().number << " with x(" << j + 1 << "," << _jobs.size() << ") = 1, LB = " << _nodes.back().lowerbound;
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
					_output << "\n\nNode " << _nodes[index_node].number << " with partial sequence ";
					for (int64_t k = _nodes[index_node].sequence.size() - 1; k >= 0; --k)
						_output << _nodes[index_node].sequence[k] + 1 << " ";
					_output << "has LB = " << _nodes[index_node].lowerbound << " >= UB = "
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


			_output << "\n\nBest remaining node is Node " << node.number << " with partial sequence ";
			for (int64_t k = node.sequence.size() - 1; k >= 0; --k)
				_output << node.sequence[k] + 1 << " ";
			_output << " and LB = " << node.lowerbound;


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

						_output << "\nComplete sequence ( ";
						for (int64_t k = node.sequence.size() - 1; k >= 0; --k)
							_output << node.sequence[k] + 1 << " ";
						_output << ") found with LB = " << node.lowerbound;

						break;
					}
				}

				if (node.lowerbound < _best_value)
				{
					_best_value = node.lowerbound;
					_best_sequence = node.sequence;

					_output << "\nNew best solution found with sequence ";
					for (int64_t k = _best_sequence.size() - 1; k >= 0; --k)
						_output << _best_sequence[k] + 1 << " ";
					_output << ", update UB = " << _best_value;
				}
			}




			// branch one level further
			else
			{
				_output << "\nBranch one level further";

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

						_output << "\nCreate a new node: Node " << _nodes.back().number << " with sequence ";
						for (int64_t k = _nodes.back().sequence.size() - 1; k >= 0; --k)
							_output << _nodes.back().sequence[k] + 1 << " ";
						_output << " and LB = " << _nodes.back().lowerbound;
					}
				}
			}
		}



		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;

		_output.set_on(true);
		_output << "\n\nElapsed time (s): " << elapsed_time.count();
		_output << "\nNodes evaluated: " << _nb_nodes;
		_output << "\nNodes pruned: " << _nb_nodes_pruned;
		_output << "\nBest sequence: ";
		for (int64_t j = _jobs.size() - 1; j >= 0; --j)
			_output << _best_sequence[j] + 1 << " "; // reverse the sequence
		_output << "\nTardiness: " << _best_value;
	}
}