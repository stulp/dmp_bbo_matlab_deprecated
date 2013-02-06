function [ task ] = task_multidofviapoint(n_dims,arm_length)

if (nargin<1), n_dims  = 10; end
if (nargin<2), arm_length = 1; end

task.n_dims = n_dims;
arm_type = 1;
task.link_lengths = getlinklengths(arm_type,n_dims,arm_length);

final_angles = ones(1,n_dims);
for ii=1:n_dims
  final_angles(ii) = pi/n_dims;
end
final_angles(1) = final_angles(1)/2.0;


task.name = 'multidofviapoint';
task.perform_rollouts = @perform_rollouts_multidofviapoint;

% Initial and goal state
task.y0 = zeros(1,n_dims);
task.g  = final_angles;

% DMP settings related to time
task.time = 0.5;
task.dt = 1/50;
task.time_exec = 0.6;

task.viapoint = [0.5 0.5]';
viapoint_time_ratio = 0.5;
task.viapoint_time_step = round(viapoint_time_ratio*task.time/task.dt);

% Minimizes accelarions
task.theta_init = repmat([37.0458   -4.2715   27.0579   13.6385],n_dims,[]);
% 
%task.theta_init = repmat([0 0],n_dims,[]);


addpath dynamicmovementprimitive/

% Now comes the function that does the roll-out and visualization thereof
  function costs = perform_rollouts_multidofviapoint(task,thetas,plot_me,color)

    K = size(thetas,2);

    for k=K:-1:1
      theta = squeeze(thetas(:,k,:));

      % Integrate the DMP
      trajectory = dmpintegrate(task.y0,task.g,theta,task.time,task.dt,task.time_exec);

      % Cost due to distance from viapoint
      angles = trajectory.y; % y represents angles
      % Get the end-effector position at time step 'viapoint_time_step'
      x_intermediate = getarmpos(angles,task.link_lengths,task.viapoint_time_step);
      % Distance of end-effector to viapoint
      dist_to_viapoint = sqrt(sum((x_intermediate-task.viapoint).^2));
      % Scale to get cost
      costs_via = dist_to_viapoint.^2;

      % Cost due to acceleration
      ydds = trajectory.ydd;
      n_dofs = task.n_dims;
      costs_acc = zeros(size(ydds,1),1);
      sum_w = 0;
      for dof=1:n_dofs %#ok<NODEF>
        %fdd = xdd(dof); % fdd = xdd[r] - up;
        costs_acc = costs_acc + (0.5*ydds(:,dof).^2)*(n_dofs+1-dof);
        sum_w = sum_w + dof;
      end
      costs_acc = 0.0000001*costs_acc/sum_w;

      % Prepare to return the costs
      costs(k,2) = costs_via;
      costs(k,3) = sum(costs_acc); % Sum over time
      % Total cost is the sum of all the subcomponent costs
      costs(k,1) = sum(costs(k,2:end));

      if (plot_me>0)
        ticks = 1:size(angles,1);
        plot_me = 1;
        if (nargin<4)
          color = 0.8*ones(1,3);
        end
        if (k==1)
          color = color*0.5;
        end
        x = getarmpos(angles,arm_length,ticks,plot_me,color);
        hold on
        plot(x(task.viapoint_time_step,1),x(task.viapoint_time_step,2),'ob')
        plot(task.viapoint(1),task.viapoint(2),'go')
      end
    end
    if (plot_me>0)
      hold off
      axis equal
    end
  end

end