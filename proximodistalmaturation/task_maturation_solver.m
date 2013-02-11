function [task_solver] = task_maturation_solver(n_dofs)

task_solver.name = 'maturation';
task_solver.perform_rollouts = @perform_rollouts_maturation_solver;
task_solver.plot_rollouts = @plot_rollouts_maturation_solver;
task_solver.plotlearninghistorycustom = @plotlearninghistorymaturation;

% Policy settings
task_solver.time = 0.5;
task_solver.time_exec = 0.6;
task_solver.dt = 1/50;
task_solver.n_dofs = n_dofs;
task_solver.n_basisfunctions = 5;
task_solver.theta_init = zeros(task_solver.n_dofs,task_solver.n_basisfunctions); % Policy parameters
task_solver.timesteps = ceil(1+task_solver.time_exec/task_solver.dt); % Number of time steps


% Pre-compute basis functions
widths = (0.4*task_solver.time/task_solver.n_basisfunctions)*ones(1,task_solver.n_basisfunctions);
centers = linspace(4*widths(1),task_solver.time-4*widths(1),task_solver.n_basisfunctions);
ts = 0:task_solver.dt:task_solver.time_exec;
%n_timesteps = length(ts);
task_solver.activations = basisfunctionactivations(centers,widths,ts);

  function plot_rollouts_maturation_solver(axes_handle,task,cost_vars)
    cla(axes_handle)

    n_time_steps = task_solver.timesteps;

    plot(task.viapoint(1),task.viapoint(2),'og')
    hold on
    n_rollouts = size(cost_vars,1);
    for k=1:n_rollouts
      angles= squeeze(cost_vars(k,:,1:3:end-1));
      plot_me = 1;
      getarmpos(angles,task.link_lengths,1:2:n_time_steps,plot_me);
      hold on
    end
    hold off
    axis([-0.3 1.1 -0.3 1.1]);
    axis equal
    drawnow
  end
    


% Now comes the function that does the roll-out and visualization thereof
  function cost_vars = perform_rollouts_maturation_solver(task,thetas) %#ok<INUSL>

    [ n_dofs n_samples n_basis_functions ]  = size(thetas); %#ok<NASGU>
    n_time_steps = task_solver.timesteps;

    cost_vars = zeros(n_samples,n_time_steps,3*n_dofs+1); % Compute n_timesteps and n_dims in constructor

    for k=1:n_samples
      theta = squeeze(thetas(:,k,:));
      
      trajectory = linearpolicyintegrate(task_solver.activations,theta,task_solver.dt);

      cost_vars(k,:,1:3:end-1) = trajectory.y;
      cost_vars(k,:,2:3:end-1) = trajectory.yd;
      cost_vars(k,:,3:3:end-1) = trajectory.ydd;
      
      cost_vars(k,:,end) = trajectory.ts;

    end
  end

end

