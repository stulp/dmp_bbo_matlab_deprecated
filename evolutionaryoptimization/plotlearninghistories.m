function plotlearninghistories(learning_histories,task,task_solver)
% Plot the history of a learning session
% Input:
%   learning_histories - the histories (see evolutionaryoptimization.m)

learning_history = learning_histories{1};
n_dofs = length(learning_history(1).distributions_new);
n_dim = length(learning_history(1).distributions_new(1).mean);

if (nargin<2)
  n_subplots = 2;
else
  n_subplots = 3;
end

for curve_type=1:2
  subplot(1,n_subplots,curve_type)
  clear curve
  for ii=1:length(learning_histories)
    learning_history = learning_histories{ii};
    for hh=1:length(learning_history)
      for i_dof=1:n_dofs %#ok<FXUP>
        if (curve_type==1)
          % Plot exploration magnitude curve (largest eigenvalue of covar)
          covar = learning_history(hh).distributions(i_dof).covar;
          curve(ii,hh) = real(max(eig(covar)));
        else
          % Plot learning curve
          curve(ii,hh) = learning_history(hh).costs(1); % First cost is noise-free evaluation trial
        end
      end
    end

    curve_avg = mean(curve,1);
    curve_std = sqrt(var(curve,1));
    plot(curve_avg,'LineWidth',2)
    hold on
    plot(curve_avg+curve_std,'LineWidth',1)
    plot(curve_avg-curve_std,'LineWidth',1)
    hold off
    axis square
    axis tight
    xlabel('number of updates')
    if (curve_type==1)
      title('Exploration magnitude over time')
      ylabel('exploration magnitude')
    else
      title('Learning curve')
      ylabel('cost of evaluation trial')
    end
  end
end

if (nargin<2)
  return
end

% Plot motion (avg +- std) as learning progresses
n_updates = length(learning_histories{1});
for uu=1:n_updates
  for hh=1:length(learning_histories)
    thetas = (learning_histories{hh}(uu).samples(:,1,:));
    cost_vars = task_solver.perform_rollouts(task,thetas);
    cost_vars_all(hh,:,:,:) = cost_vars;
  end
  cost_vars_avg = (mean(cost_vars_all,1));
  cost_vars_std = (sqrt(var(cost_vars_all,1)));
  plot_cost_vars(1,:,:,:) = cost_vars_avg;
  plot_cost_vars(2,:,:,:) = cost_vars_avg + cost_vars_std;
  plot_cost_vars(3,:,:,:) = cost_vars_avg - cost_vars_std;
  subplot(1,3,3)
  task_solver.plot_rollouts(gca,task,plot_cost_vars);
  %set(handle,'Color',[1-uu/n_updates 0.8*uu/n_updates 0]);
  title(sprintf('Motion after update %d',uu))
  pause
end

end