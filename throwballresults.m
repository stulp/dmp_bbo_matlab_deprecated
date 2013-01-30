figure(1)
clf
figure(2)
clf
for i_experiment=10:13
  filename = sprintf('result_traj_%02d.txt',i_experiment);
  data = load(filename);
  
  ball_goal = data(end,1:3);
  ball_traj = data(:,4:6);
  end_eff_traj = data(:,7:9);

  joint_pos = data(:,10:3:end);
  joint_vel = data(:,11:3:end);
  joint_acc = data(:,12:3:end);

  train_color = [0 0 1];
  test_color = [1 0 0];
  
  color = train_color;
  if (i_experiment>9)
    color = test_color;
  end

  figure(1)
  subplot(1,2,1)
  plot3(ball_goal(1),ball_goal(2),ball_goal(3),'og')
  hold on
  plot3(ball_traj(1:20:end,1),ball_traj(1:20:end,2),ball_traj(1:20:end,3),'-','Color',color)
  plot3(end_eff_traj(:,1),end_eff_traj(:,2),end_eff_traj(:,3),'-k')
  axis equal


  subplot(1,2,2)
  plot3(end_eff_traj(:,1),end_eff_traj(:,2),end_eff_traj(:,3),'-k')
  hold on
  axis equal

  figure(2)
  filename = sprintf('output_traj_%02d.txt',i_experiment-9);
  data = load(filename);
  
  joint_pos_recon = data(:,1:3:end);
  %subplot(3,1,1)
  for ii=1:7
    subplot(2,4,ii)
    plot(joint_pos(:,ii),'-r')
    hold on
    plot(joint_pos_recon(:,ii),'-b')
  end
  
  %subplot(3,1,2)
  %plot(joint_vel)
  %hold on
  %subplot(3,1,3)
  %plot(joint_acc)
  %hold on

end