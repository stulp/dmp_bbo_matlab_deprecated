function [task] = task_petanque_cb(goal_ball,g,y0)
if (nargin<1), goal_ball = zeros(1,3); end
if (nargin<2), g  = [  1.271  -0.468   0.283   1.553   0.296  -0.000   0.591  ]; end
if (nargin<3), y0   = [-0.274 -0.221  0.112  0.149 -0.001  0.001  0.104]; end

task.name = 'petanque_cb';
task.perform_rollouts = @perform_rollout_viapoint;

% Initial and goal state
task.y0 = y0;
task.g  = g;

% DMP settings related to time
task.time = 0.9;
task.dt = 1/500;
task.time_exec = 1;

task.goal_ball = goal_ball;

task.order=2; % Order of the dynamic movement primitive
% Next values optimized for minimizing acceleration in separate learning session
%task.theta_init = [37.0458   -4.2715   27.0579   13.6385; 37.0458   -4.2715   27.0579   13.6385];
task.theta_init = zeros(7,5);

[ trajectory activations canonical_at_centers ] = dmpintegrate(task.y0,task.g,task.theta_init,task.time,task.dt,task.time_exec);

% Determine scales
% Normalize
task.scales = abs(canonical_at_centers)/max(abs(canonical_at_centers));
% Avoid zeros by adding a 10% baseline
task.scales = (task.scales+0.01)/(1+0.01);


addpath dynamicmovementprimitive/

% Now comes the function that does the roll-out and visualization thereof
  function [ costs cost_vars ] = perform_rollout_viapoint(task,thetas,plot_me,color)
   
    directory = ['data_' task.name];
    write_output_to_ascii(directory,thetas,task)

    done_filename = sprintf('%s/done.txt',directory);
    if (exist(done_filename,'file'))
      delete(done_filename)
    end

    % zzz Provide functionality for writing misc else to file
    filename = sprintf('%s/goal.txt',directory);
    dlmwrite(filename,goal_ball,' ');
    
    % Run external program here
    command = './tasks/task_petanque_external_sl/runcb';
    fprintf('External program running... ');
    system(command);
    
    % Wait for the file. A crude but simple way for communication.
    while (~exist(done_filename,'file'))
      pause(0.1)
      fprintf('.');
    end
    fprintf('done.\n');
    
    all_cost_vars = read_costvars_from_ascii(directory);
    
    n_samples = size(all_cost_vars,1);
    for k=1:n_samples
      cost_vars = squeeze(all_cost_vars(k,:,:));
      
      ball_goal=  cost_vars(end,1:3);
      ball_landed =  cost_vars(end,4:6);
      
      dist = sqrt(sum((ball_landed-ball_goal).^2));
      cost = (100*dist);

      costs(k,:) = cost;

      if (plot_me)
        plot3(cost_vars(:,1),cost_vars(:,2),cost_vars(:,3),'og')
        hold on
        every = 1:20:length(cost_vars);
        plot3(cost_vars(every,4),cost_vars(every,5),cost_vars(every,6),'ok')
        plot3([ball_goal(1) ball_landed(1)],[ball_goal(2) ball_landed(2)],[ball_goal(3) ball_landed(3)],'-r')
        plot3(cost_vars(:,7),cost_vars(:,8),cost_vars(:,9),'-')
      end
      
    end
    if (plot_me)
      hold off
      axis equal
      axis tight
      zlim([-0.8 1.5])
      drawnow
    end
  end
    

end

