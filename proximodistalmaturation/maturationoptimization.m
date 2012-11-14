task = task_maturation;

K = 20;
weighting_method = 3;
eliteness = 10;
% covar_update = 0;    % Exploration does not change during learning
% covar_update = 0.9;  % Decay exploration during learning
covar_update = 2;    % Reward-weighted averaging update of covar
if (covar_update>=1)
  % For covariance matrix updating with RWA, a lower bound on the
  % eigenvalues is recommended to avoid premature convergence
  covar_bounds = [0.05 0.05 10]; %#ok<NBRAK> % Lower/upper bounds on covariance matrix
else
  covar_bounds = []; % No bounds
end

covar_init = eye(task.n_basisfunctions);

n_updates = 20;
n_experiments_per_task = 10;

if (~exist('viapoints','var'))
  viapoints = [0 1; 0 0.5; 0 0];
end
n_viapoints = size(viapoints,1);

clear learning_histories
for arm_type=1:3
  task = task_maturation(viapoints(1,:)',arm_type);

  for i_viapoint = 1:n_viapoints
    task.viapoint = viapoints(i_viapoint,:)';
    
    for i_experiment=1:n_experiments_per_task
      fprintf('arm_type=%d, viapoint=[%1.2f %1.2f] (%d/%d), i_experiment=%d\n',arm_type,task.viapoint,i_viapoint,n_viapoints, i_experiment);

      [theta_opt learning_history] = evolutionaryoptimization(task,task.theta_init,covar_init,n_updates,K,eliteness,weighting_method,covar_update,covar_bounds);
      learning_histories{arm_type,i_viapoint,i_experiment} = learning_history;
    end
  end

  figure(arm_type)
  clf
  title(arm_type);
  %task.plotlearninghistorycustom(learning_histories{arm_type,1});
  current_histories = {learning_histories{arm_type,:,:}};
  task.plotlearninghistorycustom(current_histories);
  drawnow
end

