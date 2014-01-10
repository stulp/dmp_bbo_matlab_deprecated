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

function [ task_solver ] = task_multidofviapoint_solver_dmp(n_dims,g,y0)
if (nargin<1), n_dims  = 10; end

if (nargin<2)
  final_angles = ones(1,n_dims);
  for ii=1:n_dims
    final_angles(ii) = pi/n_dims;
  end
  final_angles(1) = final_angles(1)/2.0;
  g  = final_angles;
end

if (nargin<3)
  y0 = zeros(1,n_dims);
end

task_solver.name = 'multidofviapoint';

task_solver.n_dims = n_dims;
task_solver.y0     =     y0;
task_solver.g      =      g;

task_solver.perform_rollouts = @perform_rollouts_multidofviapoint;
task_solver.plot_rollouts = @plot_rollouts_multidofviapoint_solver_dmp;

% DMP settings related to time
task_solver.time = 0.5;
task_solver.dt = 1/50;
task_solver.time_exec = 0.6;
task_solver.timesteps = ceil(1+task_solver.time_exec/task_solver.dt); % Number of time steps

%arm_type = 1;
%task.link_lengths = getlinklengths(arm_type,n_dims,arm_length);

% task.viapoint_time_step = round(viapoint_time_ratio*task.time/task.dt);

% Minimizes accelarions
task_solver.theta_init = repmat([37.0458   -4.2715   27.0579   13.6385],n_dims,[]);
% 
%task.theta_init = repmat([0 0],n_dims,[]);


addpath dynamicmovementprimitive/

  function handles = plot_rollouts_multidofviapoint_solver_dmp(axes_handle,task,cost_vars)
    cla(axes_handle)
    
    x = squeeze(cost_vars(:,:,1));
    y = squeeze(cost_vars(:,:,4));
    n_time_steps = task_solver.timesteps;
    viapoint_time_step = round(task.viapoint_time_ratio*n_time_steps);

    n_rollouts = size(cost_vars,1);
    for k=1:n_rollouts
      angles= squeeze(cost_vars(k,:,1:3:end));
      
      ticks = 1:size(angles,1);
      color = 0.8*ones(1,3);
      plot_me = 1;
      if (k==1)
        color = color*0.5;
        plot_me = 2;
      end
      x = getarmpos(angles,task.arm_length,ticks,plot_me,color);
      hold on
      plot(x(viapoint_time_step,1),x(viapoint_time_step,2),'ob')
      plot(task.viapoint(1),task.viapoint(2),'go')
    end
    hold off
    axis equal
    handles = [];
  end
    
% Now comes the function that does the roll-out and visualization thereof
  function cost_vars = perform_rollouts_multidofviapoint(task,thetas) %#ok<INUSL>

    n_samples = size(thetas,2);
    n_dims = length(task_solver.g);
    n_time_steps = task_solver.timesteps;

    cost_vars = zeros(n_samples,n_time_steps,3*n_dims); % Compute n_timesteps and n_dims in constructor
    
    for k=1:n_samples
      theta = squeeze(thetas(:,k,:));
    
      trajectory = dmpintegrate(task_solver.y0,task_solver.g,theta,task_solver.time,task_solver.dt,task_solver.time_exec);
      
      cost_vars(k,:,1:3:end) = trajectory.y;
      cost_vars(k,:,2:3:end) = trajectory.yd;
      cost_vars(k,:,3:3:end) = trajectory.ydd;
    
    end
  end

end