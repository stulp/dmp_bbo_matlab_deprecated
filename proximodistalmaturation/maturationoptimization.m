function [ learning_histories viapoints ] = maturationoptimization(link_lengths_per_arm,viapoints,n_experiments_per_task,n_updates)
if (nargin<1)
  % Arm settings
  n_dofs = 6;
  arm_length = 1;
  arm_type = 1;
  link_lengths_per_arm(1,:) = getlinklengths(arm_type,n_dofs,arm_length);
end
if (nargin<2)
  viapoints = [0 0.8];
end
if (nargin<3), n_experiments_per_task = 1; end
if (nargin<4), n_updates = 50; end

% Optimization parameters
K = 20;

n_dofs = size(link_lengths_per_arm,2);
[task_solver] = task_maturation_solver(n_dofs);

% Initial covariance matrix
n_basisfunctions = task_solver.n_basisfunctions;
covar_init = 0.05*eye(n_basisfunctions);

% Update parameters
update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
update_parameters.covar_learning_rate =       1; % No lowpass filter
update_parameters.covar_bounds        =   [0.05 0.05 10]; %#ok<NBRAK> % Lower relative bound

n_viapoints = size(viapoints,1);

n_arm_types = size(link_lengths_per_arm,1);
for arm_type=1:n_arm_types
  task = task_maturation(viapoints(1,:)',link_lengths_per_arm(arm_type,:));

  for i_viapoint = 1:n_viapoints
    task.viapoint = viapoints(i_viapoint,:)';
    
    for i_experiment=1:n_experiments_per_task
      fprintf('arm_type=%d/%d, viapoint=[%1.2f %1.2f] (%d/%d), i_experiment=%d/%d \n',arm_type,n_arm_types,task.viapoint,i_viapoint,n_viapoints, i_experiment,n_experiments_per_task);
      [theta_opt learning_history] = evolutionaryoptimization(task,task_solver,task_solver.theta_init,covar_init,n_updates,K,update_parameters);
      learning_histories{arm_type,i_viapoint,i_experiment} = learning_history;
      
      %if (mod(i_experiment,10)==1)
      %  maturationoptimizationvisualization(link_lengths_per_arm,learning_histories);
      %end

    end
  end

  if (n_experiments_per_task>1)
    maturationoptimizationvisualization(link_lengths_per_arm,learning_histories);
  end
  
end

