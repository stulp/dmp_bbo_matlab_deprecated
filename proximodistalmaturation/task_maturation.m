function [task] = task_maturation(viapoint,link_lengths)

task.name = 'maturation';
task.cost_function = @cost_function_maturation;
task.plotlearninghistorycustom = @plotlearninghistorymaturation;

% Task parameters
task.link_lengths = link_lengths;
task.n_dofs = length(link_lengths);
task.viapoint = viapoint;


% Now comes the function that does the roll-out and visualization thereof
  function costs = cost_function_maturation(task,cost_vars)

    [n_rollouts n_time_steps n_cost_vars ] = size(cost_vars); %#ok<NASGU>

    costs = zeros(n_rollouts,5);

    for k=1:n_rollouts
      ys   = squeeze(cost_vars(k,:,1:3:end-1));
      ydds = squeeze(cost_vars(k,:,3:3:end-1));

      [ n_timesteps n_dofs ] = size(ys);
      tt_intermediate = n_timesteps;


      % Cost due to acceleration
      costs_acc = zeros(n_timesteps,1);
      sum_w = 0;
      for dof=1:n_dofs
        %fdd = xdd(dof); % fdd = xdd[r] - up;
        costs_acc = costs_acc + (0.5*ydds(:,dof).^2)*(n_dofs+1-dof);
        sum_w = sum_w + dof;
      end
      costs_acc = 0.00001*costs_acc/sum_w;


      % Cost due to viapoint
      angles = ys;
      costs_via = zeros(n_timesteps,1);
      x_intermediate = getarmpos(angles,task.link_lengths,tt_intermediate);
      dist_to_viapoint = sqrt(sum((x_intermediate-viapoint).^2));
      costs_via(tt_intermediate) = 100*dist_to_viapoint.^2;

      % Cost due to joint angle limits
      costs_lim = zeros(n_timesteps,1);
      %costs_lim(trial,:) = 0.01*sum(angles,1)/n_dofs;
      %costs_lim(trial,:) = 0.01*sum(angles,1)/n_dofs;
      %x = getarmpos(angles,arm_length);
      %costs_lim(trial,:) = 0*10*((x(:,1)>task.wall(1)) & (x(:,2)>task.wall(2)));
      max_angle = max(abs(angles(:,end)));
      mean_angle = mean(abs(angles(:,end))); %#ok<NASGU>
      %disp([max_angle mean_angle])
      costs_lim(end) = max_angle;


      % Cost due to velocity at end-point
      costs_vel = zeros(n_timesteps,1);
      ts = squeeze(cost_vars(k,:,end));
      dt = ts(2);
      x = getarmpos(angles,task.link_lengths,[ n_timesteps n_timesteps-1]);
      vx = diff(x)/dt;
      v = sqrt(sum(vx.^2));

      costs_vel(end) = 0*v.^2;

      %costs = costs_acc + costs_via + costs_vel + costs_lim;
      %aggregrated_cost(k)  = sum(costs);

      costs(k,2) = sum(costs_acc);
      costs(k,3) = sum(costs_via);
      costs(k,4) = sum(costs_lim);
      costs(k,5) = sum(costs_vel);
      % Total cost is the sum of all the subcomponent costs
      costs(k,1) = sum(costs(k,2:end));

      %summary_costs = [sum(mean(costs_acc)) sum(mean(costs_vel)) sum(mean(costs_lim)) sum(mean(costs_via)) sum(mean(costs)) ];
      %summary_costs_norm  = summary_costs/sum(mean(costs));
      %fprintf('\t acc=% 3d + vel=% 3d + lim=% 3d + via=% 3d  = % 2d    ',round(100*summary_costs_norm));
      %fprintf('\t acc=%2.4f + vel=%2.4f + lim=%2.4f + via=%2.4f = %2.4f\n',(summary_costs));
    end

  end


end

