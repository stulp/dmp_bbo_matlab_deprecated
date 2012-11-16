function [ learning_histories viapoints ] = maturationoptimization(link_lengths_per_arm,viapoints,n_experiments_per_task,n_updates)
if (nargin<3), n_experiments_per_task = 10; end
if (nargin<4), n_updates = 20; end


% Optimization parameters
K = 20;
weighting_method = 3;
eliteness = 10;

% Covariance matrix updating
dummy_task = task_maturation(-1,-1); % Trick to get number of basis functions
n_basisfunctions = dummy_task.n_basisfunctions;
covar_init = eye(n_basisfunctions);
% Reward-weighted averaging update of covar. 
if (n_basisfunctions>3)
  % Diagonal only. More robust.
  covar_update = 1;
else
  % Full update.
  covar_update = 2; 
end
covar_lowpass = 0.5;

% For covariance matrix updating with RWA, a lower bound on the
% eigenvalues is recommended to avoid premature convergence
covar_bounds = [0.05 0.05 10]; %#ok<NBRAK> % Lower/upper bounds on covariance matrix

n_viapoints = size(viapoints,1);

n_arm_types = size(link_lengths_per_arm,1);
for arm_type=1:n_arm_types
  task = task_maturation(viapoints(1,:)',link_lengths_per_arm(arm_type,:));

  for i_viapoint = 1:n_viapoints
    task.viapoint = viapoints(i_viapoint,:)';
    
    for i_experiment=1:n_experiments_per_task
      fprintf('arm_type=%d/%d, viapoint=[%1.2f %1.2f] (%d/%d), i_experiment=%d/%d \n',arm_type,n_arm_types,task.viapoint,i_viapoint,n_viapoints, i_experiment,n_experiments_per_task);
      [theta_opt learning_history] = evolutionaryoptimization(task,task.theta_init,covar_init,n_updates,K,eliteness,weighting_method,covar_update,covar_bounds,covar_lowpass);
      learning_histories{arm_type,i_viapoint,i_experiment} = learning_history;
    end
  end

  if (n_experiments_per_task>1)
    maturationoptimizationvisualization(link_lengths_per_arm,learning_histories);
  end
  
end

