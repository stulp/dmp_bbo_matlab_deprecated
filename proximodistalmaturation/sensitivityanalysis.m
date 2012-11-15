function [link_lengths_per_arm perturbation_magnitude viapoints] =...
  sensitivityanalysis(link_lengths_per_arm,perturbation_magnitude,viapoints)

n_viapoints = size(viapoints,1);

clf
n_arm_types = size(link_lengths_per_arm,1);
for arm_type=1:n_arm_types 
  link_lengths = link_lengths_per_arm(arm_type,:);
  n_dofs = length(link_lengths);
  
  plot_me = 0;
  for which_angle=1:n_dofs
    angles = zeros(1,n_dofs);
    for angle_sign=1:3
      angles(which_angle) = (angle_sign-2)*(perturbation_magnitude*pi);
      x = getarmpos(angles,link_lengths,1,plot_me);
      for i_viapoint = 1:n_viapoints
        dist_to_viapoint(which_angle,angle_sign,i_viapoint) =  sqrt(sum((x-viapoints(i_viapoint,:)').^2));
      end
    end
  end
  cur_dists_plus = squeeze(dist_to_viapoint(:,1,:) - dist_to_viapoint(:,2,:));
  cur_dists_minus = squeeze(dist_to_viapoint(:,3,:) - dist_to_viapoint(:,2,:));

  subplot_handle_1 = subplot(2,n_arm_types,arm_type);
  plot_me = 2;
  plot(viapoints(:,1),viapoints(:,2),'.','Color',[0.8 0 0],'MarkerSize',12)
  for which_angle=1:n_dofs
    angles = zeros(1,n_dofs);
    for angle_sign=1:3
      angles(which_angle) = (angle_sign-2)*(perturbation_magnitude*pi);
      hold on
      x = getarmpos(angles,link_lengths,1,plot_me);
      for i_viapoint = 1:n_viapoints
        dist_to_viapoint(which_angle,angle_sign,i_viapoint) =  sqrt(sum((x-viapoints(i_viapoint,:)').^2));
      end
    end
  end
  hold off
  axis equal
  xlim([-0.1 1.1])
  ylim([-0.4 1.1])
  xlabel('x (m)')
  ylabel('y (m)')
  set(gca,'xaxisLocation','top')

  subplot_handle_2 = subplot(2,n_arm_types,n_arm_types+arm_type);
  cumsum_link_lengths = cumsum(link_lengths);
  cumsum_link_lengths = [0 cumsum_link_lengths(1:end-1)];
  for which_angle=1:n_dofs
    plot(cumsum_link_lengths(which_angle)*ones(1,2),[mean(cur_dists_minus(which_angle,:)) mean(cur_dists_plus(which_angle,:))],'LineWidth',2)
    hold on
  end
  hold off
  axis equal
  set(gca,'XTick',cumsum_link_lengths)
  set(gca,'XTickLabel',1:n_dofs)
  set(gca,'YTick',-0.2:0.2:0.2)
  xlim([-0.1 1.1])
  ylim([-0.4 0.4])
  xlabel('joint number')
  ylabel(sprintf('min/max cost with %1.2f pi perturbation',perturbation_magnitude))
  
  pos_subplot_1 = get(subplot_handle_1,'Position');
  pos_subplot_2 = get(subplot_handle_2,'Position');
  pos_subplot_2(2) = pos_subplot_1(2) - pos_subplot_2(4) + 0.5*pos_subplot_2(4);
  set(subplot_handle_1,'Position',pos_subplot_1);
  set(subplot_handle_2,'Position',pos_subplot_2);
  linkaxes([subplot_handle_1 subplot_handle_2]);
  
  if (arm_type>1)
    set(subplot_handle_1,'YTick',[]);
    set(get(subplot_handle_1,'YLabel'),'String','')
    set(subplot_handle_2,'YTick',[]);
    set(get(subplot_handle_2,'YLabel'),'String','')
  end

  drawnow


end

end


