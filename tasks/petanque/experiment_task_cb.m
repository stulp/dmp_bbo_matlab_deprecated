addpath dynamicmovementprimitive
addpath(genpath('evolutionaryoptimization/'))
addpath(genpath('tasks/'))

% Get the task to be optimized
if (~exist('goal_pos','var'))
  goal_pos = [0.0 3 -0.8];
end
task = task_petanque(goal_pos);

task_solver = task_petanque_solver_cb;

% Initial covariance matrix for exploration 
covar_init = 0.1*eye(size(task_solver.theta_init,2));
covar_init = repmat(shiftdim(covar_init,-1),7,[]);
%covar_init(3,:,:) = 300*covar_init(3,:,:);

% Number of updates, roll-outs per update
n_updates =  50;
n_samples =   5;

% Weighting method, and covariance update method
update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'decay'; % {'PI-BB','CMA-ES'}
update_parameters.covar_decay         =    0.95; 
%update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
%update_parameters.covar_learning_rate =     0.8; % No lowpass filter
update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK> 

if (isfield(task_solver,'scales'))
  update_parameters.covar_scales = task_solver.scales;
  for ii=1:size(covar_init,1)
    covar_init(ii,:,:) = squeeze(covar_init(ii,:,:))./sqrt(update_parameters.covar_scales'*update_parameters.covar_scales);
  end
end


clf
evolutionaryoptimization(task,task_solver,task_solver.theta_init,covar_init,n_updates,n_samples,update_parameters)

