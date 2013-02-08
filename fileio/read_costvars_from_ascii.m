function cost_vars = read_costvars_from_ascii(directory,current_update)
    
if (nargin<2)
  filename = sprintf('%s/current_update.txt',directory);
  current_update = load(filename);
end

output_directory = sprintf('%s/%03d_update/rollouts',directory,current_update);

filename = sprintf('%s/number_of_trials.txt',output_directory);
n_samples = load(filename);

for i_sample=1:n_samples
  filename = sprintf('%s/%02d_costvars.txt',output_directory,i_sample);
  cost_vars(i_sample,:,:) = load(filename);
end

% Increment update counter
filename = sprintf('%s/current_update.txt',directory);
dlmwrite(filename,current_update+1,' ');

end