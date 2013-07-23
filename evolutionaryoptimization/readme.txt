________________________________________________________________________________
OVERVIEW

Functions for doing black-box optimization of tasks.

The parameters of the objective function are represented as Gaussian distributions with a mean and
covariance matrix. The basic algorithm works as follows:

distribution.mean = theta_init;
distribution.covar = covar_init;

for i_update=0:n_updates

  % Get samples from the distribution 
  % theta_eps means: "mean theta + perturbation epsilon"
  theta_eps = generate_samples(distribution.mean,distribution.covar,n_samples);
  
  % Evaluate the task for these perturbed parameters to get the cost
  costs = evaluate(theta_eps)

  % Map costs to weights (lower costs should lead to higher weights
  % The sum of weights should be 1
  weights = costtoweights(costs)
  
  % Update covar with reward weighted averaging
  eps = theta_eps - distribution.mean;
  distribution.covar  = weights.*eps*eps;

  % Update mean with reward weighted averaging
  distribution.mean = weights.*theta_eps;
  
end

This is the basic algorithm underlying CMA-ES, CEM, PI^BB, (\mu_W,\lambda)-ES
The basic concepts are explained in http://www.ensta-paristech.fr/~stulp/publications/b2hd-stulp12path.html


________________________________________________________________________________
SIZE OF THE DISTRIBUTIONS

Note that distribution is an array. This is because one distribution is required for each dimension
of the DMP. Thus for DMPs:
length(distributions)         => dimensionality of the DMP (e.g. 7-D DMP for a 7-D arm)
length(distributions(1).mean) => number of basis functions in one DMP dimension

For 'standard BBO tasks' (e.g. finding the minimum of f(x) = x^2), n_dof=1, and n_dim is the problem
dimensionality (e.g. dim(x) )

Similarly covar is of size n_dof x n_dim x n_dim
For 'standard BBO tasks', the extra first dimension is added if necessary

________________________________________________________________________________
TASKS AND TASK_SOLVERS

In this code, the cost function 
  costs     = evaluate(theta_eps);
has been split into two parts
  cost_vars = task_solver.perform_rollouts(task,theta_eps); 
  costs     = task.cost_function(task,cost_vars);

The motivation for this is that multiple solvers (e.g. multiple robots or simulations thereof) may
solve the same task. The matrix "cost_vars" contains all the variables relevant to computing the
cost. 
For instance, in the simulation of the robot throwing a ball to a pre-specified location, cost_vars
should contain the location where the ball landed because this is relevant to computing the cost.

See tasks/viapoint for an example. In task_viapoint_solver_dmp.m
  function cost_vars = perform_rollouts_viapoint_solver_dmp(task,thetas)
Executes a DMP with all parameter sets in thetas, and returns the resulting trajectories in the 
"cost_vars" matrix. Then in task_viapoint.m the function
    function costs = cost_function_viapoint(task,cost_vars)
takes the "cost_vars" matrix, and computes the costs from it (by taking the sum of the
accelerations, and the distance of the trajectory to a viapoint).

To design your own task/task_solver pair, use the viapoint task as inspiration.

In case this split seems overkill, there are other very good reasons for doing so which are not
listed here (has to do with using this code to run experiments on real robots).

