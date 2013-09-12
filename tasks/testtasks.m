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

% Sanity check
% Tests whether all the tasks run correctly (not whether they do so well or not) 


% Number of updates, roll-outs per update
n_updates =  3;
n_samples =  10;

% Weighting method, and covariance update method
update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
update_parameters.covar_learning_rate =     0.8; % No lowpass filter
update_parameters.covar_bounds        =   [0.1 0.01]; %#ok<NBRAK> 

%------------------------------------------------------------
% A very simple 2-D DMP viapoint task
viapoint            = [0.4 0.7];
viapoint_time_ratio =       0.3;
task = task_viapoint(viapoint,viapoint_time_ratio);

g                   = [1.0 1.0];
y0                  = [0.0 0.0];

evaluation_external_program = 0; % This runs the evaluation of costs in an external program (i.e. not Matlab)
task_solver = task_viapoint_solver_dmp(g,y0,evaluation_external_program);
clf
covar_init = 5*eye(size(task_solver.theta_init,2)); % Initial covariance matrix for exploration
evolutionaryoptimization(task,task_solver,task_solver.theta_init,covar_init,n_updates,n_samples,update_parameters)

evaluation_external_program = 1; % This runs the evaluation of costs in an external program (i.e. not Matlab)
task_solver = task_viapoint_solver_dmp(g,y0,evaluation_external_program);
clf
evolutionaryoptimization(task,task_solver,task_solver.theta_init,covar_init,n_updates,n_samples,update_parameters)

%------------------------------------------------------------
% The JMLR style n-DOF kinmatically simulated arm task
n_dofs = 2;
arm_length = 1;
task = task_multidofviapoint(n_dofs,arm_length);
task_solver = task_multidofviapoint_solver_dmp(n_dofs);
clf
covar_init = 5*eye(size(task_solver.theta_init,2)); % Initial covariance matrix for exploration
evolutionaryoptimization(task,task_solver,task_solver.theta_init,covar_init,n_updates,n_samples,update_parameters)



