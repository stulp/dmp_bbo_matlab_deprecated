function [task] = task_viapoint_external(g,y0,viapoint,viapoint_time_ratio)
if (nargin<1), g  = [  1.271  -0.468   0.283   1.553   0.296  -0.000   0.591  ]; end
if (nargin<2), y0   = [-0.274 -0.221  0.012  0.149 -0.001  0.001  0.104]; end
%if (nargin<1), g   = [0 0]; end
%if (nargin<2), y0  = [1 1]; end
if (nargin<3), viapoint  = zeros(size(g)); end
if (nargin<4), viapoint_time_ratio = 0.5; end

task.name = 'viapoint';
task.perform_rollout = @perform_rollout_viapoint;

% Initial and goal state
task.y0 = y0;
task.g  = g;

% DMP settings related to time
task.time = 0.9;
task.dt = 1/500;
task.time_exec = 1;

task.viapoint = viapoint;
task.viapoint_time_step = round(viapoint_time_ratio*task.time/task.dt);

task.order=2; % Order of the dynamic movement primitive
% Next values optimized for minimizing acceleration in separate learning session
%task.theta_init = [37.0458   -4.2715   27.0579   13.6385; 37.0458   -4.2715   27.0579   13.6385];
task.theta_init = zeros(7,2);


addpath dynamicmovementprimitive/

% Now comes the function that does the roll-out and visualization thereof
  function costs = perform_rollout_viapoint(task,thetas,plot_me,color)
    
    if (ndims(thetas)==2)
      thetas = shiftdim(thetas,-1);
    end
    
    K = size(thetas,1);
    for k=1:K
      theta = squeeze(thetas(k,:,:));
      
      size(thetas)
      size(theta)

      trajectory = dmpintegrate(task.y0,task.g,theta,task.time,task.dt,task.time_exec);

      [ duration_ticks n_dmps ] = size(trajectory.y);

      % Put trajectory in one big array
      y_yd_ydd = zeros(duration_ticks,3*n_dmps);
      y_yd_ydd(:,1:3:end) = trajectory.y;
      y_yd_ydd(:,2:3:end) = trajectory.yd;
      y_yd_ydd(:,3:3:end) = trajectory.ydd;

      filename = sprintf('./trial%02d_dmp_output.txt',k);
      dlmwrite(filename,y_yd_ydd,' ');
      
      if (k==K)
        delete('done.txt')
        system('./runcb')
        fprintf('CB running');
        while (~exist('./done.txt','file'))
          pause(0.1)
          fprintf('.');
        end
        fprintf('\n');
      end
      
      % Cost due to distance from viapoint
      ys = trajectory.y;
      dist_to_viapoint = sqrt(sum((ys(task.viapoint_time_step,:)-viapoint).^2));
      cost(2) = dist_to_viapoint;

      % Cost due to acceleration
      ydds = trajectory.ydd;
      sum_ydd = sum((sum(ydds.^2,2)));
      cost(3) = sum_ydd/100000;

      % Total cost is the sum of all the subcomponent costs
      cost(1) = sum(cost(2:end));

      if (plot_me)
        if (nargin<4)
          color = 0.8*ones(1,3);
        end
        plot(task.viapoint(1),task.viapoint(2),'*g')
        plot(ys(:,1),ys(:,2),'Color',color)
        plot([task.viapoint(1) ys(task.viapoint_time_step,1)],[task.viapoint(2) ys(task.viapoint_time_step,2)],'Color',0.5*color)
        axis([-0.1 1.1 -0.1 1.1])
        axis equal
      end

      costs(k,:) = cost;
    end
  end

end

