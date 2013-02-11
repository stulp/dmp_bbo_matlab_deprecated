function [task] = task_petanque(goal_ball)
if (nargin<1), goal_ball = zeros(1,3); end

task.name = 'petanque';
task.goal_ball = goal_ball;
task.cost_function = @cost_function_petanque;

addpath dynamicmovementprimitive/

% Now comes the function that does the roll-out and visualization thereof
  function [ costs cost_vars ] = cost_function_petanque(task,all_cost_vars)
   
    n_samples = size(all_cost_vars,1);
    for k=1:n_samples
      cost_vars = squeeze(all_cost_vars(k,:,:));
      
      ball_goal=  cost_vars(end,1:3);
      ball_landed =  cost_vars(end,4:6);
      
      dist = sqrt(sum((ball_landed-ball_goal).^2));
      cost = (100*dist);

      costs(k,:) = cost;
    end
  end
    

end

