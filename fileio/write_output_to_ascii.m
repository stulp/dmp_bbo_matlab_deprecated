function [] = write_output_to_ascii(directory,thetas,task)
if (nargin>2)
  store_dmp_output = 1;
end

if (~exist(directory,'dir'))
  mkdir(directory);
end

filename = sprintf('%s/current_update.txt',directory);
if (~exist(filename,'file'))
  current_update = 1;
  dlmwrite(filename,current_update,' ');
else
  current_update = load(filename);
end

if (ndims(thetas)==2)
  thetas = shiftdim(thetas,-1);
end
n_samples = size(thetas,2);

output_directory = sprintf('%s/%03d_update/rollouts',directory,current_update);
if (~exist(output_directory,'dir'))
  mkdir(output_directory);
end

filename = sprintf('%s/number_of_trials.txt',output_directory);
dlmwrite(filename,n_samples,' ');

for k=1:n_samples
  theta = squeeze(thetas(:,k,:));

  filename = sprintf('%s/%02d_dmpparameters.txt',output_directory,k);
  dlmwrite(filename,theta,' ');

  if (store_dmp_output)
    trajectory = dmpintegrate(task.y0,task.g,theta,task.time,task.dt,task.time_exec);
    
    filename = sprintf('%s/%02d_traj_x.txt',output_directory,k);
    dlmwrite(filename,trajectory.y,' ');
    filename = sprintf('%s/%02d_traj_xd.txt',output_directory,k);
    dlmwrite(filename,trajectory.yd,' ');
    filename = sprintf('%s/%02d_traj_xdd.txt',output_directory,k);
    dlmwrite(filename,trajectory.ydd,' ');
  end
end
end