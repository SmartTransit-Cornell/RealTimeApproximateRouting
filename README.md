This repository contains code to replicate experiments in: 'Real-Time Approximate Routing for Smart Transit Systems' URL: https://arxiv.org/abs/2103.06212

Usage instructions: to replicate experiments for the Manhattan network (reported in table 2):

1) Download all files and unzip files manhattan_dist_1.txt, manhattan_dist_2.txt and manhattan_dist_3.txt
2) Run line_instance.py.

The experiments run by default for April 3rd 2018. To run the experiments for the fhv data from Feb 3 or March 6, uncomment the parameter 'month' line #407 in the file line_instance.py.
The code allows to test both our algorithm and the ILP solver for different values of the budget and with a time limit of 1200s. The candidate set of lines has been pregenerated and is stored in the file 'all_lines_1000_c5.txt'. The code used to generate the lines can be found in the file instance.py.




