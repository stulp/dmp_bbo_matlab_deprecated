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

% Initial and goal state
task_solver.y0 = y0;
task_solver.g  = g;

% DMP settings related to time
task_solver.time = 1;
task_solver.dt = 1/50;
task_solver.time_exec = 1.2;
task_solver.timesteps = ceil(1+task_solver.time_exec/task_solver.dt); % Number of time steps

task_solver.order=2; % Order of the dynamic movement primitive
% Next values optimized for minimizing acceleration in separate learning session
%task.theta_init = [37.0458   -4.2715   27.0579   13.6385; 37.0458   -4.2715   27.0579   13.6385];
task_solver.theta_init = zeros(2,2);

addpath dynamicmovementprimitive/

  function plot_rollouts_viapoint_solver_dmp(axes_handle,task,cost_vars)
    cla(axes_handle)
    
    x = squeeze(cost_vars(:,:,1));
    y = squeeze(cost_vars(:,:,4));
    n_time_steps = task_solver.timesteps;
    viapoint_time_step = round(task.viapoint_time_ratio*n_time_steps);

    linewidth = 1;
    color = 0.8*ones(1,3);
    plot(x(2:end,viapoint_time_step),y(2:end,viapoint_time_step),'o','Color',color,'LineWidth',linewidth)
    hold on
    plot(x(2:end,:)',y(2:end,:)','Color',color,'LineWidth',linewidth)
    my_ones = ones(size(x(2:end,viapoint_time_step)));
    plot([ x(2:end,viapoint_time_step) task.viapoint(1)*my_ones]' ,[y(2:end,viapoint_time_step) task.viapoint(2)*my_ones]','Color',color,'LineWidth',linewidth)

    linewidth = 2;
    color = 0.5*color;
    plot(x(1,viapoint_time_step),y(1,viapoint_time_step),'o','Color',color,'LineWidth',linewidth)
    plot(x(1,:)',y(1,:)','Color',color,'LineWidth',linewidth)
    plot([ x(1,viapoint_time_step) task.viapoint(1)] ,[y(1,viapoint_time_step) task.viapoint(2)],'Color',color,'LineWidth',linewidth)

    plot(task.viapoint(1),task.viapoint(2),'og')
    
    hold off
    axis([-0.1 1.1 -0.1 1.1])
  end

% Now comes the function that does the roll-out and visualization thereof
  function cost_vars = perform_rollouts_viapoint_solver_dmp(task,thetas) %#ok<INUSL>
    
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


end

