function [ learning_histories viapoints ] = maturationoptimization(link_lengths_per_arm,viapoints,n_experiments_per_task,n_updates)
if (nargin<3), n_experiments_per_task = 10; end
if (nargin<4), n_updates = 20; end

% Optimization parameters
K = 20;

% Initial covariance matrix
dummy_task = task_maturation(-1,-1); % Trick to get number of basis functions
n_basisfunctions = dummy_task.n_basisfunctions;
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
      [theta_opt learning_history] = evolutionaryoptimization(task,task.theta_init,covar_init,n_updates,K,update_parameters);
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

