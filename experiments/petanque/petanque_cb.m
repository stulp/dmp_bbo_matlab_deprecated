addpath dynamicmovementprimitive
addpath(genpath('evolutionaryoptimization/'))
addpath(genpath('tasks/'))

% Get the task to be optimized

% A very simple 2-D DMP viapoint task
g                   = [0.0 0.0];
y0                  = [1.0 1.0];
viapoint            = [0.4 0.7];
viapoint_time_ratio =       0.5;
evaluation_external_program = 0; % This runs the evaluation of costs in an external program (i.e. not Matlab)
task = task_viapoint(g,y0,viapoint,viapoint_time_ratio,evaluation_external_program);

% The JMLR style n-DOF kinmatically simulated arm task
%n_dofs = 2;
%arm_length = 1;
%task = task_multidofviapoint(n_dofs,arm_length);

if (~exist('goal_pos','var'))
  goal_pos = [0.0 3 -0.8];
end
task = task_petanque_cb(goal_pos);

% Initial covariance matrix for exploration 
covar_init = 0.1*eye(size(task.theta_init,2));
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

if (isfield(task,'scales'))
  update_parameters.covar_scales = task.scales;
  for ii=1:size(covar_init,1)
    covar_init(ii,:,:) = squeeze(covar_init(ii,:,:))./sqrt(update_parameters.covar_scales'*update_parameters.covar_scales);
  end
end

clf
evolutionaryoptimization(task,task.theta_init,covar_init,n_updates,n_samples,update_parameters)

