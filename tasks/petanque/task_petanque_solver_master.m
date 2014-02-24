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

function [task_solver] = task_petanque_solver_master(g,y0)
                       % R_SFE R_SAA  R_HR  R_EB  R_WR R_WFE R_WAA 
if (nargin<1), g    = [ -0.40 -0.30  0.00  0.70  0.00  0.00  0.00  ]+0.2; end
if (nargin<2), y0   = [  0.70 -0.30  0.00  2.00  0.00  0.00  0.00 ]; end

                       %  X   Y    Z
%if (nargin<1), g    = [ -0.40 -0.30  0.00  0.70  0.00  0.00  0.00  ]+0.1; end
%if (nargin<2), y0   = [  0.4 0.1 -0.3 ]; end
 
task_solver.name = 'petanque_master';

task_solver.perform_rollouts = @perform_rollout_petanque_solver_master;
task_solver.plot_rollouts = @plot_rollouts_petanque_solver_master;


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

  function [handles] = plot_rollouts_petanque_solver_master(axes_handle,task,all_cost_vars)
    %cla(axes_handle)

    n_samples = size(all_cost_vars,1);
    for k=1:n_samples
      cost_vars = squeeze(all_cost_vars(k,:,:));
      
      ball_goal=  cost_vars(end,1:3);
      ball_landed =  cost_vars(end,4:6);


      plot3(ball_goal(1),ball_goal(2),ball_landed(3),'og')
      hold on
      every = 1:20:length(cost_vars);
      %handles = plot3(cost_vars(every,4),cost_vars(every,5),cost_vars(every,6),'.k','Color',0.8*[1 1 1]);
      
      disp(task.goal_ball)
      ball_color = [1 1 0];
      if (task.goal_ball(end)==1)
        ball_color = [1 0 0];
      end
      if (task.goal_ball(end)==0.997)
        ball_color = [0 0 1];
      end
      handles = plot3(cost_vars(end,4),cost_vars(end,5),cost_vars(end,6),'ok','MarkerFaceColor',ball_color,'MarkerSize',5);
      plot3([ball_goal(1) ball_landed(1)],[ball_goal(2) ball_landed(2)],[ball_goal(3) ball_landed(3)],'-k')
      plot3(cost_vars(:,7),cost_vars(:,8),cost_vars(:,9),'-')
    end
    hold off
    axis equal
    axis tight
    %zlim([-1.0 1.0])
    axis([-0.5 0.5 -0.1 2 -1.0 0.5])
    view(0,90)
    drawnow
  end

% Now comes the function that does the roll-out and visualization thereof
  function [ cost_vars ] = perform_rollout_petanque_solver_master(task,thetas)
   
    n_samples = size(thetas,2);
    for k=1:n_samples
      theta = squeeze(thetas(:,k,:));
      trajectories(k) = dmpintegrate(task_solver.y0,task_solver.g,theta,task_solver.time,task_solver.dt,task_solver.time_exec);
      %plot(trajectories(k).t, trajectories(k).y)
      %hold on
    end
    %hold off
    %pause
    directory = ['./data_' task_solver.name];
    write_trajectories_to_ascii(directory,trajectories);
    
    done_filename = sprintf('%s/done.txt',directory);
    if (exist(done_filename,'file'))
      delete(done_filename)
    end

    % zzz Provide functionality for writing misc else to file
    current_update = read_current_update(directory);
    output_directory = sprintf('%s/%03d_update/rollouts',directory,current_update);
    filename = sprintf('%s/goal.txt',output_directory);
    dlmwrite(filename,task.goal_ball,' ');
    
    % Run external program here
    command = './tasks/petanque/task_petanque_external_sl/runmasterng';
    fprintf('External program running... ');
    %system(command);
    
    % Wait for the file. A crude but simple way for communication.
    while (~exist(done_filename,'file'))
      pause(0.05)
      fprintf('.');
    end
    fprintf('done.\n');
    
    cost_vars = read_costvars_from_ascii(directory);
    
  end
    

end

