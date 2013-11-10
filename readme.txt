NOTE: I am currently working on an object-oriented version of this 
code in both Matlab and C++. This should come on-line on GitHub 
early 2014. This new code will have a lot more features, and has 
already been tested much more extensively than this "dmp_bbo" code. 
So if you intend to use code like the one you can find here, I 
suggest you wait a few weeks more and use the new and improved 
version (send me an e-mail - you'll find me if you search for Freek 
Stulp -- if you want to have details on the status, or be a 
beta-tester for the upcoming code).

Since this "dmp_bbo" code is not maintained or supported anymore, 
I'll take it off-line shortly after the new release. Just so you 
know.
 
________________________________________________________________________________
WHAT THIS CODE IS FOR

This code is mainly intended to get started quickly and understand the 
basic concepts of Dynamic Movement Primitives and Black-Box Optimization 
of their shape parameters. It has been optimized for legibility, not 
performance or mass of features. For instance, it does not (yet) allow 
for the execution of policies off-line on a robot, and the evolution 
paths of CMA-ES have also not (yet) been integrated yet.


________________________________________________________________________________
HOW TO UNDERSTAND THIS CODE

1) Read the readme.txt in each directory. The best order to go throught 
the directories/files is:
  dynamicmovementprimitive/             
    -> code for integrating DMPs
  evolutionaryoptimization/ and tasks/  
    -> code for black-box optimization (independent of DMPs)
  dmp_bbo_example.m                     
    -> applying black-box optimization to DMPs

2) Almost all Matlab file have a testfunction at the bottom, which is 
called when no arguments are passed. This testfunction is essentially a 
tutorial on how to use the function. The visualizations may help to 
understand what is going on inside the function.

________________________________________________________________________________
HOW THIS CODE CAME ABOUT

Throughout 2012, I have mostly been using a 'home-made' Matlab code base 
for comparing episodic reinforcement learning (RL) algorithms (such as 
PI2) with black-box optimization (BBO) algorithms (such as CMA-ES). This 
code was used for the following papers, where we presented PI2-CMA(ES)

ICML: http://www.ensta-paristech.fr/~stulp/publications/b2hd-stulp12path.html
IROS: http://www.ensta-paristech.fr/~stulp/publications/b2hd-stulp12adaptive.html
ICDL: http://www.ensta-paristech.fr/~stulp/publications/b2hd-stulp12emergent.html

One of the main discoveries here was that black-box optimization 
outperforms episodic reinforcement learning algorithms for the tasks we 
were considering. We more thoroughly analyzed this by making slight 
modifications to another code base (in C) by Stefan Schaal (unpublished). 
The results of this analysis were published on HAL: 
http://hal.archives-ouvertes.fr/hal-00738463

The original Matlab code had grown quite big due to the many options (RL 
or BBO? covariance matrix adaptation or not? evolution paths or not? 
execute policy on robot or in-line in Matlab? etc). The realization that 
I will mainly be using BBO and not RL allows for massive simplifications. 
So I started a new code base (this one on Github), and started pulling 
functionality from the "big" Matlab code base into this one.

