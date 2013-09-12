% Author:  Freek Stulp, Robotics and Computer Vision, ENSTA-ParisTech
% Website: http://www.ensta-paristech.fr/~stulp/
% 
% Permission is granted to copy, distribute, and/or modify this program
% under the terms of the GNU General Public License, version 2 or any
% later version published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
% Public License for more details
%
% If you use this code in the context of a publication, I would appreciate 
% it if you could cite it as follows:
%
% @MISC{stulp_dmp_bbo,
%   author = {Freek Stulp},
%   title  = {dmp_bbo: Matlab library for black-box optimization of dynamical movement primitives.},
%   year   = {2013},
%   url    = {https://github.com/stulp/dmp_bbo}
% }

function evolutionaryoptimization_simple
% Simplified version of 'evolutionaryoptimization.m' to get an initial
% understanding of the code.
%
% The two main external functions this function calls are
%    Sample from a (Gaussian) distribution
%      samples = generate_samples(distribution,n_samples);
%    Update the distribution, given the samples and their costs
%      distribution = update_distributions(distribution,samples,costs,update_parameters);
%
% Two helper functions (not essential, but handy to have) are
%    Do sanity check on the parameters for updating
%      update_parameters = check_update_parameters(update_parameters);
%    Plot the learning history so far
%      plotlearninghistory(learning_history);
%
% The above functions are completely generic, and do not depend on the problem
% to be optimized at all.
%
%
% The task-specific functions are in 
%    Do a roll-out, i.e. determine the cost-relevant variables from a sample
%    (example: sample=policy parameters, cost_vars=force sensors at finger tips
%    of a robot)
%      cost_vars = task_solver.perform_rollouts(task,samples);
%
%    Compute the costs for this task, given the cost-relevant variables.
%      costs = task.cost_function(task,cost_vars);
%
% There is a very good reason for splitting the computation of the costs into
% the two functions 'perform_rollouts' and 'cost_function'. For 'standard
% optimization' of a function it is not necessary, but when working with real
% robots it is (to be explained here some day).
%
%-------------------------------------------------------------------------------

% Dimensionality of the task
n_dims = 2;

% Get the task 
% (see nested function '[task] = task_min_dist(target)' below)
target = zeros(1,n_dims);
task = task_min_dist(target);

% Get the solver for the task
% (see nested function '[task_solver] = task_solver_min_dist(task)' below)
task_solver = task_solver_min_dist(task);

% Initial parameter distribution
distribution.mean  = 5*ones(1,n_dims);
distribution.covar = 8*eye(n_dims);

% Parameters relevant to the parameter update (stored in one big structure)
% See 'update_distributions.m' for possible settings.
update_parameters.weighting_method    = 'PI-BB';
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'PI-BB'; 
update_parameters.covar_full          =       1; 
update_parameters.covar_learning_rate =       1; 
update_parameters.covar_bounds        =     0.1; % Lower relative bound
% Do sanity check on parameters, and fill with defaults where necessary.
update_parameters = check_update_parameters(update_parameters);



%-------------------------------------------------------------------------------
% Actual optimization loop
for i_update=1:20 % Do 20 updates
  fprintf('Update: %d\n',i_update);
  
  % 1. Sample from distribution
  n_samples = 10;
  samples = generate_samples(distribution,n_samples);

  % 2. Perform rollouts for the samples
  cost_vars = task_solver.perform_rollouts(task,samples);

  % 3. Evaluate the last batch of rollouts
  costs = task.cost_function(task,cost_vars);

  % 4. Update parameters 
  [ distribution update_summary ] = update_distributions(distribution,samples,costs,update_parameters);

  % Bookkeeping and plotting
  learning_history(i_update) = update_summary;
  plotlearninghistory(learning_history);
  
end

% Main function done
%-------------------------------------------------------------------------------



%-------------------------------------------------------------------------------
% Task 

  function [task] = task_min_dist(target)
    % Here is an example of how to design a task. This one simply returns the
    % distance to the target.

    task.name = 'min_dist';
    task.cost_function = @cost_function_min_dist;
    task.target = target;

    function costs = cost_function_min_dist(task,cost_vars)
      % Cost is distance to target
      n_samples = size(cost_vars,1);
      target_rep = repmat(task.target,n_samples,1);
      costs = sqrt(sum((cost_vars-target_rep).^2,2));
    end

  end

%-------------------------------------------------------------------------------
% Task solver

  function [task_solver] = task_solver_min_dist(task) %#ok<INUSD>
    % Here is an example of how to design a task solver.
    task_solver.name = 'min_dist';
    task_solver.perform_rollouts = @perform_rollouts_min_dist;
    function cost_vars = perform_rollouts_min_dist(task,samples) %#ok<INUSL>
      % For this simple optimization, 'perform_rollouts' is trivial.
      % On a robot for instance, 'samples' may be the parameters of a policy,
      % and cost_vars may be force measurements from the fingers.
      cost_vars = squeeze(samples); % Remove first dummy dimension, if necessary
    end
  end



end