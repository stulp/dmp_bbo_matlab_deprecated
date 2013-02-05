addpath dynamicmovementprimitive
addpath(genpath('evolutionaryoptimization/'))
addpath(genpath('tasks/'))

% Get the task to be optimized

% A very simple 2-D DMP viapoint task
task = task_viapoint;
% The JMLR style n-DOF kinmatically simulated arm task
%n_dofs = 2;
%arm_length = 1;
%task = task_multidofviapoint(n_dofs,arm_length);

% Initial covariance matrix for exploration 
covar_init = 5*eye(size(task.theta_init,2));

% Number of updates, roll-outs per update
n_updates =  25;
n_samples =  15;

% Weighting method, and covariance update method
update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
update_parameters.covar_learning_rate =     0.8; % No lowpass filter
update_parameters.covar_bounds        =   [0.1 0.01]; %#ok<NBRAK> 

clf
evolutionaryoptimization(task,task.theta_init,covar_init,n_updates,n_samples,update_parameters)




% NEXT STUFF IS WORK IN PROGRESS
% This is just to get the basis function activations
%[trajectory_dummy activations] = dmpintegrate(task.y0,task.g,task.theta_init,task.time,task.dt,task.time_exec);

%plot(activations)


%pol_pars.dt = pol_pars.duration/pol_pars.duration_ticks;
%[y yd ydd bases]  = integrate_dmp(pol_pars,pol_pars.mean);
%
%pol_pars.max_activation = max(abs(squeeze(bases(1,:,:))),[],2)';
%normalized_max_activation = pol_pars.max_activation/max(pol_pars.max_activation);
%pol_pars.max_activation = normalized_max_activation;
%
%for bb=1:length(pol_pars.max_activation)
%  pol_pars.covar(:,bb,bb) =  pol_pars.covar(:,bb,bb)/normalized_max_activation(bb);
%end

%return

