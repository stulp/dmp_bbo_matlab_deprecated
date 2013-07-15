function [task] = task_viapoint(viapoint,viapoint_time_ratio)
if (nargin<1), viapoint  = [0.4 0.7]; end
if (nargin<2), viapoint_time_ratio = 0.5; end

task.name = 'viapoint';

task.viapoint = viapoint;
task.viapoint_time_ratio = viapoint_time_ratio;

task.cost_function= @cost_function_viapoint;


  function costs = cost_function_viapoint(task,cost_vars)
    
    [n_rollouts n_time_steps n_cost_vars ] = size(cost_vars); %#ok<NASGU>
    viapoint_time_step = round(task.viapoint_time_ratio*n_time_steps);
    
    ys = zeros(n_time_steps,n_cost_vars/3);
    ydds = zeros(n_time_steps,n_cost_vars/3);
    for k=1:n_rollouts
      ys(:,:)   = squeeze(cost_vars(k,:,1:3:end));
      ydds(:,:) = squeeze(cost_vars(k,:,3:3:end));

      dist_to_viapoint = sqrt(sum((ys(viapoint_time_step,:)-task.viapoint).^2));
      costs(k,2) = dist_to_viapoint;

      % Cost due to acceleration
      sum_ydd = sum((sum(ydds.^2,2)));
      costs(k,3) = sum_ydd/100000;

      % Total cost is the sum of all the subcomponent costs
      costs(k,1) = sum(costs(k,2:end));
    end
  end

end

