addpath dynamicmovementprimitive
addpath(genpath('evolutionaryoptimization/'))
addpath(genpath('tasks/'))

% Get the task to be optimized
g   = [0 0];
y0  = [1 -1];
g_distract = g + [0.5 0.0];
n_dim = length(g);

for switch_g = 0:1
  if (switch_g)
    g_distract = g - [0.5 0.0];
  end
  task = task_predictme(g,y0,g_distract);
  
  n_learning_sessions = 5;
  clear learning_histories
  figure(1)
  for i_learning_session=1:n_learning_sessions
    fprintf('Learning session %d%d',i_learning_session,n_learning_sessions)

    % This is the task solver
    task_solver = task_predictme_solver(g,y0);

    % Initial covariance matrix for exploration
    covar_init = 5*eye(size(task_solver.theta_init,2));
    covar_init = repmat(shiftdim(covar_init,-1),n_dim,[]);

    % Number of updates, roll-outs per update
    n_updates =   20;
    n_samples =   15;

    % Weighting method, and covariance update method
    update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
    update_parameters.eliteness           =      10;
    update_parameters.covar_update        = 'decay'; % {'PI-BB','CMA-ES'}
    update_parameters.covar_decay         =     0.9;
    %update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
    %update_parameters.covar_learning_rate =     0.8; % No lowpass filter
    update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK>

    clf
    [theta_opt learning_history] = evolutionaryoptimization(task,task_solver,task_solver.theta_init,covar_init,n_updates,n_samples,update_parameters);

    learning_histories{i_learning_session} = learning_history;
  end

  figure(2+switch_g)
  plotlearninghistories(learning_histories,task,task_solver)
end