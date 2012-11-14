figure(1)
sensitivityanalysis;

figure(2)
if (~exist('results_uncertaintyhandling','var'))
  results_uncertaintyhandling = uncertaintyhandling(10);
else
  results_uncertaintyhandling = uncertaintyhandling(results_uncertaintyhandling);
end

if (~exist('learning_history','var'))
  figure(3)
  learning_history = maturationoptimization;
end

% Plot exploration magnitude curve (largest eigenvalue of covar)
figure(4)
n_dofs = length(task.link_lengths);
for i_dof=1:n_dofs
  legend_labels{i_dof} = sprintf('%d',i_dof);
  for hh=1:length(learning_history)
    exploration_curve(i_dof,hh) = real(max(eig(squeeze(learning_history(hh).covar(i_dof,:,:)))));
  end  
end
plot(exploration_curve');
legend(legend_labels)

