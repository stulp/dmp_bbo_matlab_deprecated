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

function [task_solver] = task_viapoint_solver_dmp(g,y0,evaluation_external_program)
task_solver.name = 'viapoint';
if (nargin<1), g   = [1 1]; end
if (nargin<2), y0  = [0 0]; end
if (nargin<3), evaluation_external_program = 0; end

if (evaluation_external_program)
  task_solver.perform_rollouts = @perform_rollouts_viapoint_solver_dmp_external;
  addpath(genpath('fileio/'))
else
  task_solver.perform_rollouts = @perform_rollouts_viapoint_solver_dmp;
end

task_solver.plot_rollouts = @plot_rollouts_viapoint_solver_dmp;

% This is not needed for the dmp_bbo code, it is needed for another code base.
% Please ignore this.
task_solver.observation_function = @observation_function_viapoint_solver_dmp;

% Initial and goal state
task_solver.y0 = y0;
task_solver.g  = g;
task_solver.n_dim = length(y0);

% DMP settings related to time
task_solver.time = 1;
task_solver.dt = 1/50;
task_solver.time_exec = 1.5;
task_solver.n_time_steps = ceil(1+task_solver.time/task_solver.dt); % Number of time steps
task_solver.n_time_steps_exec = ceil(1+task_solver.time_exec/task_solver.dt); % Number of time steps

task_solver.order=2; % Order of the dynamic movement primitive
task_solver.theta_init = zeros(task_solver.n_dim,2);

addpath dynamicmovementprimitive/

  function line_handles = plot_rollouts_viapoint_solver_dmp(axes_handle,task,cost_vars) %#ok<INUSL>
    %cla(axes_handle)
    
    [ n_rollouts n_time_steps n_cost_vars ] = size(cost_vars);
    viapoint_time_step = round(task.viapoint_time_ratio*n_time_steps);

    if (task_solver.n_dim==1)
      x = repmat(1:n_time_steps,n_rollouts,1);
      y = squeeze(cost_vars(:,:,1));
      x_via = viapoint_time_step;
      y_via = task.viapoint(1);
      
    elseif (task_solver.n_dim==2)
      x = squeeze(cost_vars(:,:,1));
      y = squeeze(cost_vars(:,:,4));
      x_via = task.viapoint(1);
      y_via = task.viapoint(2);
      
    else
      warning('Only know how to plot rollouts for viapoint task when n_dim<3, but n_dim==%d',task_solver.n_dim); %#ok<WNTAG>
      return;
    end
    
    color = 0.8*ones(1,3);
    linewidth = 1;
    
    % Draw trajectory
    line_handles(1,:) =  plot(x',y','Color',color,'LineWidth',linewidth);
    hold on
    % Draw line from trajectory to viapoint
    if (task_solver.n_dim==2)
      my_ones = ones(size(x(:,viapoint_time_step)));
      line_handles(2,:) = plot([ x(:,viapoint_time_step) x_via*my_ones]' ,[y(:,viapoint_time_step) y_via*my_ones]','Color',color,'LineWidth',linewidth);
    end
    % Highlight position along trajectory at viapoint_time_step
    %line_handles(3,:) =  plot(x(:,viapoint_time_step),y(:,viapoint_time_step),'o','Color',color,'LineWidth',linewidth);
    
    % Plot viapoint
    plot(x_via,y_via,'o','MarkerFaceColor',[0.6 0.9 0.6],'MarkerSize',8,'MarkerEdgeColor',0.5*[1 1 1]);
    hold off
    
    axis square
    axis tight
    
    if (task_solver.n_dim==1)
      ylim([-0.1 1.1])
    else
      axis([-0.1 1.1 -0.1 1.1])
    end 
  end

% Now comes the function that does the roll-out and visualization thereof
  function cost_vars = perform_rollouts_viapoint_solver_dmp(task,thetas) %#ok<INUSL>

    n_samples = size(thetas,2);
    n_basis_functions = size(thetas,3);
    n_dims = length(task_solver.g);

    cost_vars = zeros(n_samples,task_solver.n_time_steps_exec,3*n_dims); % Compute n_time_steps and n_dims in constructor

    theta = zeros(n_dims,n_basis_functions);
    for k=1:n_samples

      theta(:,:) = squeeze(thetas(:,k,:));
      
      trajectory = dmpintegrate(task_solver.y0,task_solver.g,theta,task_solver.time,task_solver.dt,task_solver.time_exec,task_solver.order);

      cost_vars(k,:,1:3:end) = trajectory.y;
      cost_vars(k,:,2:3:end) = trajectory.yd;
      cost_vars(k,:,3:3:end) = trajectory.ydd;

    end

  end

% Now comes the function that does the roll-out and visualization thereof
  function cost_vars = perform_rollouts_viapoint_solver_dmp_external(task,thetas)

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

    % Run external program here
    command = ['./tasks/viapoint/task_viapoint_external_cpp/task_viapoint_external_cpp ' pwd '/' directory];
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


  function observation = observation_function_viapoint_solver_dmp(task,N,min_values,max_values,figure_handle) %#ok<DEFNU>
    n_dim = length(task.viapoint);
    if (n_dim~=2)
      error('Sorry. observation_function_viapoint only works for n_dim==2 (but it is %d)',n_dim)
    end
    if (nargin<2), N = 20; end
    if (nargin<3), min_values = zeros(1,n_dim); end
    if (nargin<4), max_values = ones(1,n_dim); end
    if (nargin<5), figure_handle = 0; end

    % Scale viapoint in normalized space [0-1]
    scaled_viapoint = (task.viapoint-min_values)./(max_values-min_values);
    % Generate X/Y grid in normalized space
    [X Y] = meshgrid(linspace(0,1,N),linspace(0,1,N));
    % Get value in multi-variate normal distribution
    Z = mvnpdf([ X(:) Y(:)],scaled_viapoint,0.005*eye(n_dim));
    % Make an NxN image of this.
    image = reshape(Z,N,N);

    max_image = max(max(image));
    image(image<0.01*max_image) = 0;
    image = image + 2*randn(size(image));

    if (figure_handle)
      figure(figure_handle)
      mesh(image)
      axis square
      axis tight
      colormap(gray)
    end

    observation = image(:);


  end

end

