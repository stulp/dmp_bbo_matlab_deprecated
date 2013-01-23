function [task] = task_viapoint_external(goal_ball,g,y0)
if (nargin<1), goal_ball = zeros(1,3); end
if (nargin<2), g  = [  1.271  -0.468   0.283   1.553   0.296  -0.000   0.591  ]; end
if (nargin<3), y0   = [-0.274 -0.221  0.112  0.149 -0.001  0.001  0.104]; end

task.name = 'viapoint';
task.perform_rollout = @perform_rollout_viapoint;

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
task.scales = abs(canonical_at_centers)/max(abs(canonical_at_centers))
% Avoid zeros by adding a 10% baseline
task.scales = (task.scales+0.01)/(1+0.01)


addpath dynamicmovementprimitive/

% Now comes the function that does the roll-out and visualization thereof
  function [ costs cost_vars ] = perform_rollout_viapoint(task,thetas,plot_me,color)
    
    filename = sprintf('./data/current_update.txt');
    current_update = load(filename);

    if (ndims(thetas)==2)
      thetas = shiftdim(thetas,-1);
    end
    
    K = size(thetas,1);
    
    mkdir('data',sprintf('%03d_txt_files',current_update));
    filename = sprintf('./data/%03d_txt_files/number_of_trials.txt',current_update);
    dlmwrite(filename,K,' ');

    filename = sprintf('./data/%03d_txt_files/goal.txt',current_update);
    dlmwrite(filename,goal_ball,' ');

    
    for k=1:K
      theta = squeeze(thetas(k,:,:));
      
      trajectory = dmpintegrate(task.y0,task.g,theta,task.time,task.dt,task.time_exec);

      [ duration_ticks n_dmps ] = size(trajectory.y);

      % Put trajectory in one big array
      y_yd_ydd = zeros(duration_ticks,3*n_dmps);
      y_yd_ydd(:,1:3:end) = trajectory.y;
      y_yd_ydd(:,2:3:end) = trajectory.yd;
      y_yd_ydd(:,3:3:end) = trajectory.ydd;

      filename = sprintf('./data/%03d_txt_files/trial%02d_dmp_output.txt',current_update ,k);
      dlmwrite(filename,y_yd_ydd,' ');
    end
    
    
    delete('./data/done.txt')
    system('./runcb')
    fprintf('CB running');
    while (~exist('./data/done.txt','file'))
      pause(0.1)
      fprintf('.');
    end
    fprintf('\n');

    
    for k=1:K
      filename = sprintf('./data/%03d_txt_files/trial%02d_cost_vars.txt',current_update ,k);
      cost_vars = load(filename);
      
      ball_goal=  cost_vars(end,1:3);
      ball_landed =  cost_vars(end,4:6);
      
      dist = sqrt(sum((ball_landed-ball_goal).^2));
      cost = (100*dist)^2;

      costs(k,:) = cost;

      if (plot_me)
        plot3(cost_vars(:,1),cost_vars(:,2),cost_vars(:,3),'og')
        hold on
        every = 1:20:length(cost_vars);
        plot3(cost_vars(every,4),cost_vars(every,5),cost_vars(every,6),'ok')
        plot3([ball_goal(1) ball_landed(1)],[ball_goal(2) ball_landed(2)],[ball_goal(3) ball_landed(3)],'-r')
        plot3(cost_vars(:,7),cost_vars(:,8),cost_vars(:,9),'-')
        axis equal
        axis tight
        zlim([-0.8 1.5])
        if (k==K)
          hold off
        end
      end
      
    end
  end

end

