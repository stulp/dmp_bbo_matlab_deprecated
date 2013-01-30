clear goal_pos_all;

goal_pos_all = [];
for dx = [-0.4 0 0.4]
  for dy = [-0.4 0 0.4]
    goal_pos_all(end+1,:) = [0+dx 3+dy -0.8];
  end
end
for dx = [-0.2 0.2]
  for dy = [-0.2 0.2]
    goal_pos_all(end+1,:) = [0+dx 3+dy -0.8];
  end
end

goal_pos_all
n_experiments = size(goal_pos_all,1)
pause


for i_experiment = 1:n_experiments
  goal_pos = goal_pos_all(i_experiment,:);
  filename = sprintf('result_%02d.mat',i_experiment);
  if (exist(filename,'file'))
    load(filename);
    task = task_viapoint_external(goal_pos);
  else
    dmp_bbo_example;
    save(filename,'goal_pos','theta_opt','learning_history')
  end

  if (i_experiment==1)
    [ costs cost_vars ] = task.perform_rollout(task,task.theta_init,1);
    ball_landed =  cost_vars(end,4:6);

    figure(70)
    clf
    plot(ball_landed(1),ball_landed(2),'*m')
    hold on
    figure(1)
  end

  [ costs cost_vars ] = task.perform_rollout(task,theta_opt,1);
  ball_goal =  cost_vars(end,1:3);
  ball_landed =  cost_vars(end,4:6);

  figure(70)
  subplot(1,2,1)
  plot(goal_pos(1),goal_pos(2),'*g')
  hold on
  plot(ball_goal(1),ball_goal(2),'ok')
  plot(ball_landed(1),ball_landed(2),'or')
  plot([goal_pos(1) ball_landed(1)],[goal_pos(2) ball_landed(2)],'-k')
  axis equal
  axis ([-1 1 3-1 3+1])

  subplot(1,2,2)
  plot3(cost_vars(:,7),cost_vars(:,8),cost_vars(:,9),'-k')
  hold on
  %plot(goal_pos(1),goal_pos(2),-0.8,'*g')
  filename = sprintf('result_traj_%02d.txt',i_experiment);
  dlmwrite(filename,cost_vars,' ');

  pause(3)
  figure(1)
end
figure(70)
hold off


n_experiments = count-1;
for count=1:n_experiments
  load(sprintf('result_%02d.mat',count))
  theta_opt
end