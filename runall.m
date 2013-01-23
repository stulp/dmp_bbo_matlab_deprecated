count = 1;
for dx = [0 -0.4 0.4]
  for dy = [0 -0.4 0.4]
    goal_pos = [0+dx 3+dy -0.8];
    dmp_bbo_example;
    save(sprintf('result_%02d.mat',count),'goal_pos','theta_opt','learning_history')
    
    if (count==1)
      [ costs cost_vars ] = task.perform_rollout(task,task.theta_init,1);
      ball_landed =  cost_vars(end,4:6);

      figure(70)
      plot(ball_landed(1),ball_landed(2),'*m')
      hold on
      figure(1)
    end
    count = count+1;
    
    [ costs cost_vars ] = task.perform_rollout(task,theta_opt,1);

    size(costs)
    size(cost_vars)
    
    ball_goal=  cost_vars(end,1:3)
    ball_landed =  cost_vars(end,4:6);

    figure(70)
    plot(goal_pos(1),goal_pos(2),'*g')
    hold on
    plot(ball_goal(1),ball_goal(2),'ok')
    plot(ball_landed(1),ball_landed(2),'or')
    plot([goal_pos(1) ball_landed(1)],[goal_pos(2) ball_landed(2)],'-k')
    axis equal
    axis ([-1 1 3-1 3+1])
    pause(3)
    figure(1)
    if (count==1)
      return
    end
  end
end
figure(70)
hold off