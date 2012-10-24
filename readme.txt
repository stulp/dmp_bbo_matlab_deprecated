How to understand this code:

1) Read the readme.txt in each directory. The best order to go throught the 
directories/files is:
  dynamicmovementprimitive/             - code for integrating DMPs
  evolutionaryoptimization/ and tasks/  - code for black-box optimization (independent of DMPs and dynamicmovementprimitive/)
  dmp_bbo_example.m                     - applying black-box optimization to DMPs

2) Almost all Matlab file have a testfunction at the bottom, which is called when 
no arguments are passed. This testfunction is essentially a tutorial on how to use 
the function. The visualizations may help to understand what is going on inside 
the function.
