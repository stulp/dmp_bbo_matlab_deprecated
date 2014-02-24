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

addpath dynamicmovementprimitive
addpath(genpath('evolutionaryoptimization/'))
addpath(genpath('tasks/'))

% Get the task to be optimized
if (~exist('goal_pos','var'))
  goal_pos = [-0.4 1.5 -0.95];
end
task = task_petanque(goal_pos);

task_solver = task_petanque_solver_master;

% Initial covariance matrix for exploration 
covar_init = 0.1*eye(size(task_solver.theta_init,2));
covar_init = repmat(shiftdim(covar_init,-1),7,[]);
%covar_init(3,:,:) = 300*covar_init(3,:,:);

% Number of updates, roll-outs per update
n_updates =  50;
n_samples =  10;

% Weighting method, and covariance update method
update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.covar_decay         =    0.95; 
update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
update_parameters.covar_learning_rate =     0.8; % No lowpass filter
update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK> 
update_parameters = check_update_parameters(update_parameters);

if (isfield(task_solver,'scales'))
  update_parameters.covar_scales = task_solver.scales;
  for ii=1:size(covar_init,1)
    covar_init(ii,:,:) = squeeze(covar_init(ii,:,:))./sqrt(update_parameters.covar_scales'*update_parameters.covar_scales);
  end
end


clf
evolutionaryoptimization(task,task_solver,task_solver.theta_init,covar_init,n_updates,n_samples,update_parameters)

