function [task] = task_viapoint(g,y0,viapoint,viapoint_time_ratio,evaluation_external_program)
if (nargin<1), g   = [0 0]; end
if (nargin<2), y0  = [1 1]; end
if (nargin<3), viapoint  = [0.4 0.7]; end
if (nargin<4), viapoint_time_ratio = 0.5; end
if (nargin<5), evaluation_external_program = 0; end


task.name = 'viapoint';
if (evaluation_external_program)
  task.perform_rollouts = @perform_rollouts_viapoint_external;
  addpath(genpath('fileio/'))
else
  task.perform_rollouts = @perform_rollouts_viapoint;
end

% Initial and goal state
task.y0 = y0;
task.g  = g;

% DMP settings related to time
task.time = 1;
task.dt = 1/50;
task.time_exec = 1.2;

task.viapoint = viapoint;
task.viapoint_time_step = round(viapoint_time_ratio*task.time/task.dt); 

task.order=2; % Order of the dynamic movement primitive
% Next values optimized for minimizing acceleration in separate learning session
%task.theta_init = [37.0458   -4.2715   27.0579   13.6385; 37.0458   -4.2715   27.0579   13.6385];
task.theta_init = zeros(2,2);


addpath dynamicmovementprimitive/

  function costs = evaluate_costs(task,ys,yds,ydds,plot_me,k)

    dist_to_viapoint = sqrt(sum((ys(task.viapoint_time_step,:)-viapoint).^2));
    costs(2) = dist_to_viapoint;

    % Cost due to acceleration
    sum_ydd = sum((sum(ydds.^2,2)));
    costs(3) = sum_ydd/10000;

    % Total cost is the sum of all the subcomponent costs
    costs(1) = sum(costs(2:end));

    if (nargin>4 && plot_me)
      linewidth = 1;
      color = 0.8*ones(1,3);
      if (nargin>5 && k==1)
        % Draw first rollout a bit differently
        color=0.5*color;
        linewidth = 3;
      end
      plot(ys(:,1),ys(:,2),'Color',color,'LineWidth',linewidth)
      plot([task.viapoint(1) ys(task.viapoint_time_step,1)],[task.viapoint(2) ys(task.viapoint_time_step,2)],'Color',color)
    end

  end

% Now comes the function that does the roll-out and visualization thereof
  function costs = perform_rollouts_viapoint(task,thetas,plot_me,color)
    
    K = size(thetas,2);
    
    for k=K:-1:1
      theta = squeeze(thetas(:,k,:));
    
      trajectory = dmpintegrate(task.y0,task.g,theta,task.time,task.dt,task.time_exec);

      costs(k,:) = evaluate_costs(task,trajectory.y,trajectory.yd,trajectory.ydd,plot_me,k);

    end

    if (plot_me)
      plot(task.viapoint(1),task.viapoint(2),'og')
      axis([-0.1 1.1 -0.1 1.1])
    end      

  end

% Now comes the function that does the roll-out and visualization thereof
  function [ costs ] = perform_rollouts_viapoint_external(task,thetas,plot_me,color)
    
    directory = ['data_' task.name];
    write_output_to_ascii(directory,thetas,task)

    done_filename = sprintf('%s/done.txt',directory);
    if (exist(done_filename,'file'))
      delete(done_filename)
    end

    % Run external program here
    command = ['./tasks/task_viapoint_external_cpp/task_viapoint_external_cpp ' pwd '/' directory];
    fprintf('External program running... ');
    system(command);
    
    % Wait for the file. A crude but simple way for communication.
    while (~exist(done_filename,'file'))
      pause(0.1)
      fprintf('.');
    end
    fprintf('done.\n');

    all_cost_vars = read_costvars_from_ascii(directory);

    n_samples = size(thetas,2);
    for k=n_samples:-1:1
      ys   = squeeze(all_cost_vars(k,:,1:3:end));
      yds  = squeeze(all_cost_vars(k,:,2:3:end));
      ydds = squeeze(all_cost_vars(k,:,3:3:end));
    
      costs(k,:) = evaluate_costs(task,ys,yds,ydds,plot_me,k);

    end

    if (plot_me)
      plot(task.viapoint(1),task.viapoint(2),'og')
      axis([-0.1 1.1 -0.1 1.1])
    end      
    
  end


end

