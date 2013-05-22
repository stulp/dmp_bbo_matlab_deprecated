function [task_solver] = task_predictme_solver(g,y0)
task_solver.name = 'predictme';

task_solver.perform_rollouts = @perform_rollouts_predictme_solver;
task_solver.plot_rollouts = @plot_rollouts_predictme_solver;

% Initial and goal state
task_solver.y0 = y0;
task_solver.g  = g;

% DMP settings related to time
task_solver.time = 1;
task_solver.dt = 1/50;
task_solver.time_exec = 1.5;
task_solver.timesteps = ceil(1+task_solver.time_exec/task_solver.dt); % Number of time steps

task_solver.order=2; % Order of the dynamic movement primitive
% Next values optimized for minimizing acceleration in separate learning session
task_solver.theta_init = [37.0458   -4.2715   27.0579   13.6385; 37.0458   -4.2715   27.0579   13.6385];

addpath dynamicmovementprimitive/

% Now comes the function that does the roll-out 
  function cost_vars = perform_rollouts_predictme_solver(task,thetas) %#ok<INUSL>
    
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

  function handle = plot_rollouts_predictme_solver(axes_handle,task,cost_vars)
    cla(axes_handle)
    
    x = squeeze(cost_vars(:,:,1));
    y = squeeze(cost_vars(:,:,4));

    linewidth = 1;
    color = 0.8*ones(1,3);
    plot(x(2:end,:)',y(2:end,:)','Color',color,'LineWidth',linewidth)
    hold on

    linewidth = 2;
    color = 0.5*color;
    handle = plot(x(1,:)',y(1,:)','Color',color,'LineWidth',linewidth);

    plot(task.g(1),task.g(2),'*g');
    plot(task.g_distract(1),task.g_distract(2),'*r');

    hold off
  end

end

