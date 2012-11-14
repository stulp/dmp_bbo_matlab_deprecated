function [task] = task_maturation(viapoint,arm_type)
if (nargin<1), viapoint  = [0.5 0.5]'; end
if (nargin<2), arm_type = 2; end

task.name = 'maturation';
task.perform_rollout = @perform_rollout_maturation;
task.plotlearninghistorycustom = @plotlearninghistorymaturation;

% Policy settings
task.time = 0.5;
task.time_exec = 0.6;
task.dt = 1/50;
task.n_dofs = 6;
n_basisfunctions = 2;
task.n_basisfunctions = n_basisfunctions;
task.theta_init = zeros(task.n_dofs,task.n_basisfunctions); % Policy parameters

% Pre-compute basis functions
widths = (0.4*task.time/n_basisfunctions)*ones(1,n_basisfunctions);
centers = linspace(4*widths(1),task.time-4*widths(1),n_basisfunctions);
ts = 0:task.dt:task.time_exec;
%n_timesteps = length(ts);
task.activations = basisfunctionactivations(centers,widths,ts);

% Task parameters
arm_length=1;
task.link_lengths = getlinklengths(arm_type,task.n_dofs,arm_length);
task.viapoint = viapoint;


% Now comes the function that does the roll-out and visualization thereof
  function aggregrated_cost = perform_rollout_maturation(task,theta,plot_me,color)

    trajectory = linearpolicyintegrate(task.activations,theta,task.dt);

    [ n_timesteps n_dofs ] = size(trajectory.y);
    tt_intermediate = n_timesteps;

    costs = zeros(n_timesteps,1);
    costs_acc = costs;
    costs_via = costs;
    costs_vel = costs;
    costs_lim = costs;

    % Cost due to acceleration
    sum_w = 0;
    for dof=1:n_dofs
      %fdd = xdd(dof); % fdd = xdd[r] - up;
      costs_acc = costs_acc + (0.5*trajectory.ydd(:,dof).^2)*(n_dofs+1-dof);
      sum_w = sum_w + dof;
    end
    costs_acc = 0.00001*costs_acc/sum_w;


    % Cost due to viapoint
    angles = trajectory.y;
    x_intermediate = getarmpos(angles,task.link_lengths,tt_intermediate);
    dist_to_viapoint = sqrt(sum((x_intermediate-viapoint).^2));
    costs_via(tt_intermediate) = 100*dist_to_viapoint.^2;

    % Cost due to joint angle limits
    %costs_lim(trial,:) = 0.01*sum(angles,1)/n_dofs;
    %costs_lim(trial,:) = 0.01*sum(angles,1)/n_dofs;
    %x = getarmpos(angles,arm_length);
    %costs_lim(trial,:) = 0*10*((x(:,1)>task.wall(1)) & (x(:,2)>task.wall(2)));
    max_angle = max(abs(angles(:,end)));
    mean_angle = mean(abs(angles(:,end))); %#ok<NASGU>
    %disp([max_angle mean_angle])
    costs_lim(end) = max_angle;


    % Cost due to velocity at end-point
    x = getarmpos(angles,task.link_lengths,[ n_timesteps n_timesteps-1]);
    vx = diff(x)/task.dt;
    v = sqrt(sum(vx.^2));

    costs_vel(end) = 0*v.^2;

    costs = costs_acc + costs_via + costs_vel + costs_lim;
    aggregrated_cost  = sum(costs);

    summary_costs = [sum(mean(costs_acc)) sum(mean(costs_vel)) sum(mean(costs_lim)) sum(mean(costs_via)) sum(mean(costs)) ];
    summary_costs_norm  = summary_costs/sum(mean(costs));
    fprintf('\t acc=% 3d + vel=% 3d + lim=% 3d + via=% 3d  = % 2d    ',round(100*summary_costs_norm));
    fprintf('\t acc=%2.4f + vel=%2.4f + lim=%2.4f + via=%2.4f = %2.4f\n',(summary_costs));

    if (plot_me)
      getarmpos(trajectory.y,task.link_lengths,n_timesteps,plot_me);
      hold on
      plot(task.viapoint(1),task.viapoint(2),'*g')
      axis([-0.3 1 -0.3 1]);
      axis equal
    end


  end

  function plotlearninghistorymaturation(learning_history)
    n_updates = length(learning_history);
    if (n_updates==1)
      clf
    end

    n_dofs = size(learning_history(1).theta,1);
    exploration_curves = zeros(n_updates,n_dofs);
    for i_dof=1:n_dofs %#ok<FXUP>
      for hh=1:length(learning_history)
        exploration_curves(hh,i_dof) = real(max(eig(squeeze(learning_history(hh).covar(i_dof,:,:))))); % HACK
      end
    end

    for ii=1:n_dofs
      labels{ii} = sprintf('joint %d',ii);
    end

    cumsum_exploration_curves = cumsum(exploration_curves,2);

    %----------------------------------------------------------
    % Set colormap to have 'n_dofs' entries
    colormap(ones(n_dofs,3))
    map = colormap(jet);
    %c =[ 1:ceil(n_dofs/2); (floor(n_dofs/2)+1):n_dofs]
    %map = map(c(:),:);
    map = rot90(rot90(map));
    %map(end,:) = 1;
    set(gcf,'DefaultAxesColorOrder',map)
    colormap(map);

    %----------------------------------------------------------
    % Raw data
    subplot(1,2,1)
    plot(exploration_curves,'LineWidth',2)
    legend(labels,'Location','EastOutside')
    axis tight

    %----------------------------------------------------------
    % Normalized and plot as patches
    subplot(1,2,2)
    patch_pad = cumsum_exploration_curves;
    for ii=n_dofs:-1:1
      patch([1:n_updates n_updates 1 1],[ patch_pad(:,ii)' 0 0 patch_pad(1,ii)],map(ii,:),'EdgeColor','none')
      hold on
    end
    hold off
    legend(labels{end:-1:1},'Location','EastOutside')
    axis tight
    title('Exploration magnitude over time')
    xlabel('number of updates')
    ylabel('exploration magnitude')
    
  end

end

