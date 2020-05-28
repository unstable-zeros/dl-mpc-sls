# dl-mpc-cdc2020

The code needed to reproduce the examples in 

C. Amo Alonso, and N. Matni, [Distributed and Localized Closed Loop Model Predictive Control via System Level Synthesis](https://arxiv.org/abs/1909.10074)

is in folder DLMPC.


The code needed to reproduce the examples in 

C. Amo Alonso, N. Matni, and J. Anderson, [Explicit Distributed and Localized Model Predictive Control via System Level Synthesis](TBD)

is in folder explicit_DLMPC.


Each of the scnearios/cases shown in each of the figures is simulated individually, with a script named with the scenario/case number (open loop is named "scenario0"). To reproduce the figures in the paper, once the script is run the data must be saved in a .mat file named the same as the corresponding .m script. After all the scenarios in a folder have been simulated and the corresponding data have been run, "ploting_code.m" produces the figures shown in the paper.  

*Warning*: some of the scripts, in particular the ones concerning runtime measures, might take several hours to run.
