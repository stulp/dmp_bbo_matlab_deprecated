function [] = write_trajectories_to_ascii(directory,trajectories)

if (~exist(directory,'dir'))
  [status,message,messageid] = mkdir(directory); %#ok<NASGU>
end

current_update = read_current_update(directory);

output_directory = sprintf('%s/%03d_update/rollouts',directory,current_update);
if (~exist(output_directory,'dir'))
  mkdir(output_directory);
end

n_samples = length(trajectories);
filename = sprintf('%s/number_of_trials.txt',output_directory);
dlmwrite(filename,n_samples,' ');

for k=1:n_samples

  trajectory = trajectories(k);
    
  filename = sprintf('%s/%02d_traj_x.txt',output_directory,k);
  dlmwrite(filename,trajectory.y,' ');
  filename = sprintf('%s/%02d_traj_xd.txt',output_directory,k);
  dlmwrite(filename,trajectory.yd,' ');
  filename = sprintf('%s/%02d_traj_xdd.txt',output_directory,k);
  dlmwrite(filename,trajectory.ydd,' ');

end
end