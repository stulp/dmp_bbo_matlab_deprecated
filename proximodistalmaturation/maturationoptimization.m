task = task_maturation;

n_updates = 10;
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

[theta_opt learning_history] = evolutionaryoptimization(task,task.theta_init,covar_init,n_updates,K,eliteness,weighting_method,covar_update,covar_bounds);

