% Author:  Freek Stulp, Robotics and Computer Vision, ENSTA-ParisTech
% Website: http://www.ensta-paristech.fr/~stulp/
% 
% Permission is granted to copy, distribute, and/or modify this program
% under the terms of the GNU General Public License, version 2 or any
% later version published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
% Public License for more details
%
% If you use this code in the context of a publication, I would appreciate 
% it if you could cite it as follows:
%
% @MISC{stulp_dmp_bbo,
%   author = {Freek Stulp},
%   title  = {dmp_bbo: Matlab library for black-box optimization of dynamical movement primitives.},
%   year   = {2013},
%   url    = {https://github.com/stulp/dmp_bbo}
% }

function plotlearninghistory(learning_history,highlight,main_color)
% Plot the history of a learning session
% Input:
%   learning_history - the history (see evolutionaryoptimization.m)
%   highlight        - 1: plot the samples, 0: keep it simple (default)

if (nargin==0), testplotlearninghistory; return; end
if (nargin<2), highlight=0; end
if (nargin<3), main_color=[0 0 0.8]; end

n_dofs = length(learning_history(1).distributions_new);
n_dims = length(learning_history(1).distributions_new(1).mean);

% Very difficult to see anything in the plots for many dofs
plot_n_dofs = min(n_dofs,3);


% If there are more thatn 2 degrees of freedom, the subplots get cluttered. So
% remove annotations
annotate_plots = (n_dofs<3); 

%my_colormap = repmat(linspace(0.8,0.2,length(path)),3,1)';

% Generate a circle for plotting
% zzz Should be marker, as in update_distributions_visualization
aaa = linspace(0,2*pi,20);
circle = 3*[sin([aaa(end) aaa])' cos([aaa(end) aaa])'];

% zzz highlight != plot_cost_to_weight

for i_dof=1:plot_n_dofs %#ok<FXUP>

  if (highlight)
    figure(1)
    clf
    subplot(1,2,2)    
    box on
    subplot(1,2,1)    
    box on
  else
    subplot(plot_n_dofs,4,(i_dof-1)*4 + 2)
    cla
  end
  
  % Plot only most recent history entries
  n_most_recent_summaries = 10;
  
  from_summary = max(1,length(learning_history)-n_most_recent_summaries+1);
  colors = [linspace(0.95,main_color(1),n_most_recent_summaries)'...
            linspace(0.95,main_color(2),n_most_recent_summaries)'...
            linspace(0.95,main_color(3),n_most_recent_summaries)'...
            ];
          
  n_summaries = min(length(learning_history),n_most_recent_summaries);
  for hh=1:n_summaries
    %disp([ hh  length(learning_history)-n_summaries+hh ]);
    
    highlight = (hh==n_summaries);
    plot_samples = (hh==n_summaries);
    i_color  = hh+(n_most_recent_summaries-n_summaries);
    color = colors(i_color,:);
    summary = learning_history(end-n_summaries+hh);
    if (n_dims<3)
      update_distributions_visualize(summary,highlight,plot_samples,i_dof,color)
    end

    axis square
    axis equal
    if (annotate_plots)
      xlabel('\theta_1')
      ylabel('\theta_2')
    end
    
  end
  highlight=0; % zzz
  
  costs = learning_history(hh).costs;
  weights = learning_history(hh).weights;
  if (highlight)
    axis_range_1 = [range(xlim) range(ylim)];
    subplot(1,2,2)
    box on
    %axis_range_2 = [range(costs) range(weights)]
    axis_range_2 = [1.3*range(costs) 0.6];
    circle_scale = axis_range_2./axis_range_1;
    
    for k=1:size(theta_eps,1)
      % zzz patch(costs(k)+circle_scale(1)*weights(k)*circle(:,1),weights(k)+circle_scale(2)*weights(k)*circle(:,2),[0.7 1 0.7],'EdgeColor','none')
    end
    plot(sort(costs),sort(weights,'descend'),'-','Color',0.8*ones(1,3))
    hold on
    plot(sort(costs),sort(weights,'descend'),'o','MarkerFaceColor',[0.5 0.5 1.0],'MarkerEdgeColor','k')
    hold off

    % Reposition subplots
    subplot_handle_1 = subplot(1,2,1);

    subplot(1,2,2)
    axis tight
    xlim([min(costs)-0.15*range(costs) max(costs)+0.15*range(costs)])
    ylim([0 axis_range_2(2)])
    set(gca,'Position',[0.1 0.1 0.7 0.7])
    axis square
    xlabel('costs')
    ylabel('weights')

    set(subplot_handle_1,'Position',[0.22 0.21 0.7 0.7])

    %plot2svg(sprintf('plotlearninghistory%02d.svg',up_to))
  else

    subplot(plot_n_dofs,4,(i_dof-1)*4 + 3)

    % Plot exploration magnitude curve (largest eigenvalue of covar)
    for hh=1:length(learning_history)
      covar = learning_history(hh).distributions(i_dof).covar;
      exploration_curve(hh,:) = real(max(eig(covar))); % HACK
    end
    plot(exploration_curve,'LineWidth',2,'Color',main_color)
    axis tight
    axis square
    ylim([0 max(exploration_curve)])
    x_ticks_exploration = get(gca,'XTick');
    if (annotate_plots)
      title('Exploration magnitude over time')
      xlabel('number of updates')
      ylabel('exploration magnitude')
    end
  end
  
end

if (highlight)
  return;
end



% Plot learning curves
%std_costs_exploration = [];
all_costs = [];
costs_mean = [];
n_rollouts_per_update = [];
for hh=1:length(learning_history)
  all_costs = [all_costs; learning_history(hh).costs];
  costs_mean(hh,:) = mean(learning_history(hh).costs);
  n_rollouts_per_update(hh) = size(learning_history(hh).costs,1);
  %std_costs_exploration(hh,:) = sqrt(var(learning_history(hh).costs));
end

subplot(plot_n_dofs,4,4:4:plot_n_dofs*4)
plot(all_costs(:,1),'.','Color',0.8*ones(1,3))
hold on
first_is_mean=0;
if (first_is_mean)
  evaluation_rollouts = cumsum([1 n_rollouts_per_update(1:end-1)]);
  plot(evaluation_rollouts,all_costs(evaluation_rollouts,:),'-','LineWidth',1)
  plot(evaluation_rollouts,all_costs(evaluation_rollouts,1),'-','LineWidth',2,'Color',main_color)
  if (length(x_ticks_exploration)<=length(evaluation_rollouts))
    set(gca,'XTick',evaluation_rollouts(x_ticks_exploration));
    set(gca,'XTickLabel',x_ticks_exploration);
  end
else
  evaluation_rollouts = cumsum(n_rollouts_per_update(1:end));
  plot(evaluation_rollouts,costs_mean,'-','LineWidth',1)
  plot(evaluation_rollouts,costs_mean(:,1),'-','LineWidth',2,'Color',main_color)
  if (length(x_ticks_exploration)<=length(evaluation_rollouts))
    set(gca,'XTick',evaluation_rollouts(x_ticks_exploration));
    set(gca,'XTickLabel',x_ticks_exploration);
  end
end

hold off
axis square
axis tight
%xlim([1 n_updates])
title('Learning curve')
legend('cost (all rollouts)','cost (noise-free rollouts)')
xlabel('number of updates')
%xlabel('number of evaluations')
ylabel('costs')

drawnow

  function testplotlearninghistory
    n_dofs = 1;
    n_samples=20;
    n_updates = 15;
    for n_basisfunctions=2:4
      clear learning_history;

      for i_dof=1:n_dofs %#ok<FXUP>
        node.distributions(i_dof).mean = 10*ones(1,n_basisfunctions);
        node.distributions(i_dof).covar = eye(n_basisfunctions,n_basisfunctions);
      end
      learning_history(1) = node;
      
      for uu=1:n_updates
        learning_history(uu).samples = generate_samples(learning_history(uu).distributions,n_samples);
        costs = zeros(n_samples,1);
        for i_dof=1:n_dofs %#ok<FXUP>
          learning_history(uu).costs = costs + sum(squeeze(learning_history(uu).samples(i_dof,:,:)),2);
          % Fake an update
          learning_history(uu).weights = 1./learning_history(uu).costs;
          learning_history(uu).distributions_new(i_dof).mean  = 0.8*learning_history(uu).distributions(i_dof).mean;
          learning_history(uu).distributions_new(i_dof).covar = 0.8*learning_history(uu).distributions(i_dof).covar;
        end
        
        learning_history(uu+1).distributions = learning_history(uu).distributions_new; 
      end
      learning_history = learning_history(1:n_updates);
        
      figure(n_basisfunctions)
      plotlearninghistory(learning_history)
    end

  end

end
