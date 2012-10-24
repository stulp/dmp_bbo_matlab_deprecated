Functions for doing black-box optimization of tasks

The 'evolutionaryoptimization' takes a task (see below), an initial theta and covariance matrix, and some other arguments.
The parameters 'theta' are then optimized so that they minize the cost of the task.


A note about the structure of theta and covar:

theta is n_dof x n_dim
With DMPs, n_dof would typically be the number of transformation systems, and n_dim the number of basis functions per transformation system.
For 'standard BBO tasks' (e.g. finding the minimum of f(x) = x^2), n_dof=1, and n_dim is the problem dimensionality (e.g. |x|)

Similarly covar is of size n_dof x n_dim x n_dim
For 'standard BBO tasks', the extra first dimension is added if necessary




A task is a structure that must contain a function 'task.perform_rollout'. This function should have the signature
  cost = perform_rollout(task,theta,plot_me,color)
where 
   task    - the task itself (to have access to its parameters), 
   theta   - the parameter vector with which the rollout is performed
   plot_me - wether to visualize the rollout (default: do not plot)
   color   - color of the visualizer rollout (handy to set from outside)