function [] = write_dmp_parameters_to_ascii(directory,thetas)

if (~exist(directory,'dir'))
  mkdir(directory);
end

current_update = read_current_update(directory);

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

end
end