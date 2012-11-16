function [task] = task_maturation(viapoint,link_lengths)

task.name = 'maturation';
task.perform_rollout = @perform_rollout_maturation;
task.plotlearninghistorycustom = @plotlearninghistorymaturation;

% Task parameters
task.link_lengths = link_lengths;
task.viapoint = viapoint;

% Policy settings
task.time = 0.5;
task.time_exec = 0.6;
task.dt = 1/50;
task.n_dofs = length(link_lengths);
n_basisfunctions = 5;
task.n_basisfunctions = n_basisfunctions;
task.theta_init = zeros(task.n_dofs,task.n_basisfunctions); % Policy parameters

% Pre-compute basis functions
widths = (0.4*task.time/n_basisfunctions)*ones(1,n_basisfunctions);
centers = linspace(4*widths(1),task.time-4*widths(1),n_basisfunctions);
ts = 0:task.dt:task.time_exec;
%n_timesteps = length(ts);
task.activations = basisfunctionactivations(centers,widths,ts);



% Now comes the function that does the roll-out and visualization thereof
  function aggregrated_cost = perform_rollout_maturation(task,theta,plot_me,color)

    trajectory = linearpolicyintegrate(task.activations,theta,task.dt);

    [ n_timesteps n_dofs ] = size(trajectory.y);
    tt_intermediate = n_timesteps;

    costs = zeros(n_timesteps,1);
    costs_acc = costs;
    costs_via = costs;
    costs_vel = costs;
    costs_lim = costs;

    % Cost due to acceleration
    sum_w = 0;
    for dof=1:n_dofs
      %fdd = xdd(dof); % fdd = xdd[r] - up;
      costs_acc = costs_acc + (0.5*trajectory.ydd(:,dof).^2)*(n_dofs+1-dof);
      sum_w = sum_w + dof;
    end
    costs_acc = 0.00001*costs_acc/sum_w;


    % Cost due to viapoint
    angles = trajectory.y;
    x_intermediate = getarmpos(angles,task.link_lengths,tt_intermediate);
    dist_to_viapoint = sqrt(sum((x_intermediate-viapoint).^2));
    costs_via(tt_intermediate) = 100*dist_to_viapoint.^2;

    % Cost due to joint angle limits
    %costs_lim(trial,:) = 0.01*sum(angles,1)/n_dofs;
    %costs_lim(trial,:) = 0.01*sum(angles,1)/n_dofs;
    %x = getarmpos(angles,arm_length);
    %costs_lim(trial,:) = 0*10*((x(:,1)>task.wall(1)) & (x(:,2)>task.wall(2)));
    max_angle = max(abs(angles(:,end)));
    mean_angle = mean(abs(angles(:,end))); %#ok<NASGU>
    %disp([max_angle mean_angle])
    costs_lim(end) = max_angle;


    % Cost due to velocity at end-point
    x = getarmpos(angles,task.link_lengths,[ n_timesteps n_timesteps-1]);
    vx = diff(x)/task.dt;
    v = sqrt(sum(vx.^2));

    costs_vel(end) = 0*v.^2;

    costs = costs_acc + costs_via + costs_vel + costs_lim;
    aggregrated_cost  = sum(costs);

    summary_costs = [sum(mean(costs_acc)) sum(mean(costs_vel)) sum(mean(costs_lim)) sum(mean(costs_via)) sum(mean(costs)) ];
    summary_costs_norm  = summary_costs/sum(mean(costs));
    %fprintf('\t acc=% 3d + vel=% 3d + lim=% 3d + via=% 3d  = % 2d    ',round(100*summary_costs_norm));
    %fprintf('\t acc=%2.4f + vel=%2.4f + lim=%2.4f + via=%2.4f = %2.4f\n',(summary_costs));

    if (plot_me)
      if (plot_me==2)
        getarmpos(trajectory.y,task.link_lengths,1:2:n_timesteps,plot_me);
      else
        getarmpos(trajectory.y,task.link_lengths,n_timesteps,plot_me);
      end
      hold on
      plot(task.viapoint(1),task.viapoint(2),'*g')
      axis([-0.3 1 -0.3 1]);
      axis equal
    end


  end


end

