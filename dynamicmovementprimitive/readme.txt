Implementation of discrete Dynamic Movement Primitives

Calling functions without any arguments tests them and plots some things. 

To integrate, there are two options:

1) Integrate the system in one go. This is implemented by for instance "canonicalintegrate.m"

2) Integrate the system step-by-step. This is implemented by for instance "canonicalintegratestep.m", and "canonicalreset.m"

On robots, you will typically want to do 2), because 1) does not allow for feedback, i.e. you are integrating the system before executing it on the robot.
In simple Matlab simulations (where no perturbations arise), it is quicker to integrate the system in one go.

TODO: Implement "dmpintegratestep.m", and "dmpreset.m" (i.e. option 2) for DMPs)


