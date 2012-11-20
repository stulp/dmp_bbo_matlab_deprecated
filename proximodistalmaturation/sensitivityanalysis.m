function [link_lengths_per_arm perturbation_magnitude viapoints] =...
  sensitivityanalysis(link_lengths_per_arm,perturbation_magnitude,viapoints)

n_viapoints = size(viapoints,1);

clf
n_arm_types = size(link_lengths_per_arm,1);
for arm_type=1:n_arm_types
  link_lengths = link_lengths_per_arm(arm_type,:);
  cumsum_link_lengths = [0 cumsum(link_lengths)];
  n_dofs = length(link_lengths);

  costs = zeros(n_dofs,2,n_viapoints);

  subplot_handle_1 = subplot(2,n_arm_types,arm_type);
  plot(viapoints(:,1),viapoints(:,2),'.','Color',[0.8 0 0],'MarkerSize',12)
  plot_me = 2;
  for which_angle=1:n_dofs
    angles = zeros(1,n_dofs);

    for angle_sign=1:2
      angles(which_angle) = (angle_sign-1)*(perturbation_magnitude*pi);

      % Compute end-effector for these angles
      hold on
      x = getarmpos(angles,link_lengths,1,plot_me);

      % Cost due to joint angle limits
      max_angle = max(abs(angles(:,end)));
      mean_angle = mean(abs(angles(:,end))); %#ok<NASGU>
      costs_lim = max_angle;

      for i_viapoint = 1:n_viapoints

        % Cost due to viapoint
        dist_to_viapoint = sqrt(sum((x-viapoints(i_viapoint,:)').^2));
        costs_via = 100*dist_to_viapoint.^2;

        % Total cost for this
        costs(which_angle,angle_sign,i_viapoint) =  costs_lim  + costs_via;

      end

    end

  end

  hold off
  axis equal
  xlim([-0.1 1.1])
  ylim([-0.1 1.1])
  xlabel('x (m)')
  ylabel('y (m)')
  set(gca,'xaxisLocation','top')

  % Difference between default and perturbed posture
  diff_costs = squeeze(-diff(costs,1,2));
  % Mean over all viapoints
  mean_diff_costs = mean(diff_costs,2);

  % Plot mean cost for this angle
  subplot_handle_2 = subplot(2,n_arm_types,n_arm_types+arm_type);
  %bar(cumsum_link_lengths(1:end-1),mean_diff_costs,0.2);
  colormap(ones(n_dofs,3))
  map = colormap(jet);
  map = rot90(rot90(map));
  for i_dof=1:n_dofs
    plot(cumsum_link_lengths(i_dof)*ones(1,2),[0 mean_diff_costs(i_dof)],'LineWidth',5,'Color',map(i_dof,:))
    hold on
  end
  hold off

  set(gca,'XTick',cumsum_link_lengths(1:end-1))
  set(gca,'XTickLabel',1:n_dofs)
  %set(gca,'YTick',-0.2:0.2:0.2)
  ylim([0 30])
  xlim([-0.1 1.1])
  axis square
  xlabel('joint number')
  ylabel('mean cost difference')

  pos_subplot_1 = get(subplot_handle_1,'Position');
  pos_subplot_2 = get(subplot_handle_2,'Position');
  pos_subplot_2(2) = pos_subplot_1(2) - pos_subplot_2(4) + 0.5*pos_subplot_2(4);
  %set(subplot_handle_1,'Position',pos_subplot_1);
  %set(subplot_handle_2,'Position',pos_subplot_2);
  %linkaxes([subplot_handle_1 subplot_handle_2]);

  if (arm_type>1)
    set(subplot_handle_1,'YTick',[]);
    set(get(subplot_handle_1,'YLabel'),'String','')
    set(subplot_handle_2,'YTick',[]);
    set(get(subplot_handle_2,'YLabel'),'String','')
  end

  drawnow


end

end


