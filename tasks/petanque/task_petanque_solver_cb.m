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

function [task_solver] = task_petanque_solver_cb(g,y0)
if (nargin<1), g  = [  1.271  -0.468   0.283   1.553   0.296  -0.000   0.591  ]; end
if (nargin<2), y0   = [-0.274 -0.221  0.112  0.149 -0.001  0.001  0.104]; end

task_solver.name = 'petanque_cb';

task_solver.perform_rollouts = @perform_rollout_petanque_solver_cb;
task_solver.plot_rollouts = @plot_rollouts_petanque_solver_cb;


% Initial and goal state
task_solver.y0 = y0;
task_solver.g  = g;

% DMP settings related to time
task_solver.time = 0.9;
task_solver.dt = 1/500;
task_solver.time_exec = 1;

task_solver.order=2; % Order of the dynamic movement primitive
% Next values optimized for minimizing acceleration in separate learning session
%task.theta_init = [37.0458   -4.2715   27.0579   13.6385; 37.0458   -4.2715   27.0579   13.6385];
task_solver.theta_init = zeros(7,5);

[ trajectory activations canonical_at_centers ] = dmpintegrate(task_solver.y0,task_solver.g,task_solver.theta_init,task_solver.time,task_solver.dt,task_solver.time_exec);

% Determine scales
% Normalize
task_solver.scales = abs(canonical_at_centers)/max(abs(canonical_at_centers));
% Avoid zeros by adding a 10% baseline
task_solver.scales = (task_solver.scales+0.01)/(1+0.01);


addpath dynamicmovementprimitive/

  function plot_rollouts_petanque_solver_cb(axes_handle,task,all_cost_vars)
    cla(axes_handle)

    n_samples = size(all_cost_vars,1);
    for k=1:n_samples
      cost_vars = squeeze(all_cost_vars(k,:,:));
      
      ball_goal=  cost_vars(end,1:3);
      ball_landed =  cost_vars(end,4:6);


      plot3(ball_goal(1),ball_goal(2),ball_landed(3),'og')
      hold on
      every = 1:20:length(cost_vars);
      plot3(cost_vars(every,4),cost_vars(every,5),cost_vars(every,6),'ok')
      plot3([ball_goal(1) ball_landed(1)],[ball_goal(2) ball_landed(2)],[ball_goal(3) ball_landed(3)],'-k')
      plot3(cost_vars(:,7),cost_vars(:,8),cost_vars(:,9),'-')
    end
    hold off
    axis equal
    axis tight
    zlim([-0.8 1.5])
    drawnow
  end

% Now comes the function that does the roll-out and visualization thereof
  function [ cost_vars ] = perform_rollout_petanque_solver_cb(task,thetas)
   
    n_samples = size(thetas,2);
    for k=1:n_samples
      theta = squeeze(thetas(:,k,:));
      trajectories(k) = dmpintegrate(task_solver.y0,task_solver.g,theta,task_solver.time,task_solver.dt,task_solver.time_exec);
    end
    directory = ['./data_' task.name];
    write_trajectories_to_ascii(directory,trajectories);

    done_filename = sprintf('%s/done.txt',directory);
    if (exist(done_filename,'file'))
      delete(done_filename)
    end

    % zzz Provide functionality for writing misc else to file
    filename = sprintf('%s/goal.txt',directory);
    dlmwrite(filename,task.goal_ball,' ');
    
    % Run external program here
    command = './tasks/petanque/task_petanque_external_sl/runcb';
    fprintf('External program running... ');
    system(command);
    
    % Wait for the file. A crude but simple way for communication.
    while (~exist(done_filename,'file'))
      pause(0.1)
      fprintf('.');
    end
    fprintf('done.\n');
    
    cost_vars = read_costvars_from_ascii(directory);
    
  end
    

end

