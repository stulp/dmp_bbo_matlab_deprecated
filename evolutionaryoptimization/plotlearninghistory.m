function plotlearninghistory(learning_history,highlight)
% Plot the history of a learning session
% Input:
%   learning_history - the history (see evolutionaryoptimization.m)
%   highlight        - 1: plot the samples, 0: keep it simple (default)

if (nargin==0), testplotlearninghistory; return; end
if (nargin<2), highlight=0; end


n_dofs = length(learning_history(1).distributions_new);
n_dim = length(learning_history(1).distributions_new(1).mean);


% If there are more thatn 2 degrees of freedom, the subplots get cluttered. So
% remove annotations
annotate_plots = (n_dofs<3); 

%my_colormap = repmat(linspace(0.8,0.2,length(path)),3,1)';

% Generate a circle for plotting
aaa = linspace(0,2*pi,20);
circle = 3*[sin([aaa(end) aaa])' cos([aaa(end) aaa])'];

for i_dof=1:n_dofs %#ok<FXUP>

  if (highlight)
    figure(1)
    clf
    subplot(1,2,2)    
    box on
    subplot(1,2,1)    
    box on
  else
    subplot(n_dofs,4,(i_dof-1)*4 + 2)
    cla
  end
  
  % Plot only most recent 10 history entries
  for hh=[ 1 max(1,length(learning_history)-10):length(learning_history) ]

    theta = learning_history(hh).distributions(i_dof).mean;
    covar = learning_history(hh).distributions(i_dof).covar;
    costs = learning_history(hh).costs;
    theta_new = learning_history(hh).distributions_new(i_dof).mean;
    covar_new = learning_history(hh).distributions_new(i_dof).covar;
    if (highlight)
      theta_eps = learning_history(hh).samples;
      weights  = learning_history(hh).weights;
    end
    
    plot_n_dim = min(n_dim,3); % Plot only first two dimensions

    if (hh==length(learning_history) && highlight)
      for k=1:size(theta_eps,1)
        theta_k = theta_eps(i_dof,k,:);
        if (plot_n_dim==2)
          % Green circle representing weight
          patch(theta_k(1)+weights(k)*circle(:,1),theta_k(2)+weights(k)*circle(:,2),[0.7 1 0.7],'EdgeColor','none')
          hold on
          % Line from current mean to theta_k
          plot([theta(1) theta_k(1)],[theta(2) theta_k(2)],'-','Color',[0.5 0.5 1.0])
          % theta_k
          plot(theta_k(1),theta_k(2),'o','MarkerFaceColor',[0.5 0.5 1.0],'MarkerEdgeColor','k')
        elseif (plot_n_dim==3)
          warning('Cannot plot green circle representing weight in 3 dimensions') %#ok<WNTAG>
          % Line from current mean to theta_k
          plot([theta(1) theta_k(1)],[theta(2) theta_k(2)],[theta(3) theta_k(3)],'-','Color',[0.5 0.5 1.0])
          hold on
          % theta_k
          plot(theta_k(1),theta_k(2),theta_k(3),'o','MarkerFaceColor',[0.5 0.5 1.0],'MarkerEdgeColor','k')
        end

      end
      % Line from current to new theta
      plot([theta(1) theta_new(1)],[theta(2) theta_new(2)],'-','Color',0*[0.5 0 1],'LineWidth',3)
    end

    if (plot_n_dim==2)
      h_before_theta = plot(theta(1),    theta(2)    ,'o','MarkerSize',10,'MarkerEdgeColor','none');
      hold on
      h_after_theta  = plot(theta_new(1),theta_new(2),'o','MarkerSize',10,'MarkerEdgeColor','none');
    elseif (plot_n_dim==3)
      h_before_theta = plot3(theta(1),    theta(2)    ,    theta(3),'o','MarkerSize',10,'MarkerEdgeColor','none');
      hold on
      h_after_theta  = plot3(theta_new(1),theta_new(2),theta_new(3),'o','MarkerSize',10,'MarkerEdgeColor','none');
    end
      
    % Note that plotting this might lead to numerical issues because we are
    % taking a submatrix of the covariance matrix
    h_before_covar = error_ellipse(real(squeeze(covar(1:plot_n_dim,1:plot_n_dim))),theta(1:plot_n_dim));
    h_after_covar  = error_ellipse(real(squeeze(covar_new(1:plot_n_dim,1:plot_n_dim))),theta_new(1:plot_n_dim));

    if (hh==length(learning_history))
      set([h_before_covar h_after_covar],'LineWidth',2)
      set(h_before_theta ,'MarkerFaceColor',[0.2 0.2 0.7],'MarkerEdgeColor','k')
      set(h_after_theta ,'MarkerFaceColor',[0.9 0.2 0.2],'MarkerEdgeColor','k')
      if (plot_n_dim<3)
        set(h_before_covar,'Color',[0.2 0.2 0.7]);
        set(h_after_covar,'Color',[0.9 0.2 0.2]);
      end
    else
      if (plot_n_dim<3)
        set([h_before_covar h_after_covar],'Color',0.8*ones(1,3),'LineWidth',1);
        set(h_before_theta ,'MarkerFaceColor',0.8*ones(1,3),'MarkerEdgeColor','none');
      end
    end


    axis square
    axis equal
    %axis([-7 17 -7 17])
    %plot(0,0,'*k')
    if (annotate_plots)
      xlabel('\theta_1')
      ylabel('\theta_2')
    end
    
  end
  
  if (highlight)
    axis_range_1 = [range(xlim) range(ylim)];
    subplot(1,2,2)
    box on
    %axis_range_2 = [range(costs) range(weights)]
    axis_range_2 = [1.3*range(costs) 0.6];
    circle_scale = axis_range_2./axis_range_1;
    
    for k=1:size(theta_eps,1)
      patch(costs(k)+circle_scale(1)*weights(k)*circle(:,1),weights(k)+circle_scale(2)*weights(k)*circle(:,2),[0.7 1 0.7],'EdgeColor','none')
      hold on
    end
    plot(sort(costs),sort(weights,'descend'),'-','Color',0.8*ones(1,3))
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

    subplot(n_dofs,4,(i_dof-1)*4 + 3)

    % Plot exploration magnitude curve (largest eigenvalue of covar)
    for hh=1:length(learning_history)
      covar = learning_history(hh).distributions(i_dof).covar;
      exploration_curve(hh,:) = real(max(eig(covar))); % HACK
    end
    plot(exploration_curve)
    axis square
    ylim([0 max(exploration_curve)])
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
std_costs_exploration = [];
for hh=1:length(learning_history)
  %exploit_times(hh) = length(costs_exploration)+1;
  costs_exploitation(hh,:) = learning_history(hh).costs(1);
  std_costs_exploration(hh,:) = sqrt(var(learning_history(hh).costs));
end

subplot(n_dofs,4,4:4:n_dofs*4)
plot(costs_exploitation,'-','LineWidth',2)
hold on
plot(costs_exploitation(:,1)+std_costs_exploration(:,1),'-b');
plot(costs_exploitation(:,1)-std_costs_exploration(:,1),'-b');
hold off
axis square
%xlim([1 n_updates])
title('Learning curve')
legend('cost (total)')
xlabel('number of updates')
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
