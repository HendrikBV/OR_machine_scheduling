Algorithms for the Single Machine Scheduling Problem
====================================================
This software implements several algorithms for the single machine scheduling problem. There is is a set of jobs with durations and due dates. The objective is to determine the sequence in which the jobs should be processed on the machine such that the total tardiness of the jobs is minimized.

Example on how to run the software:
`OR_machine_scheduling --data="example.txt" --algorithm="BB" --verbose`

Parameters:
* `--algorithm`  The choice of algorithm. Possibilities:
  + "CE": complete enumeration
  + "BB": a dedicated branch-and-bound approach
  + "CPLEX1": an integer programming model solved with CPLEX (with variables x[j][k] = 1 if job j is at position k in the sequence, 0 otherwise) 
  + "CPLEX2": an alternative integer programming model solved with CPLEX (with variables z[i][j] = 1 if job i comes before job j in the sequence, 0 otherwise)
  + "IP1": an integer programming model solved with SCIP (with variables x[j][k] = 1 if job j is at position k in the sequence, 0 otherwise) 
  + "IP2": an alternative integer programming model solved with SCIP (with variables z[i][j] = 1 if job i comes before job j in the sequence, 0 otherwise)
* `--data`       Name of the file containing the problem data
* `--verbose`        Explain the various steps of the algorithm
* `--help`         Help on how to use the application