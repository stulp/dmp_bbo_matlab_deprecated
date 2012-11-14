function sensitivityanalysis

%[ task pol_pars_shared pi_pars_shared pi_meta_pars viz_pars ] = initpimeta_icdl2012;

viapoint_xs = linspace(0,1,9);
viapoint_ys = linspace(0.25,1.,9);
n_viapoints = length(viapoint_xs) * length(viapoint_ys); %#ok<NASGU>

n_arm_types = getlinklengths;
clf
for arm_type=1:n_arm_types 
  link_lengths = getlinklengths(arm_type);
  n_dofs = length(link_lengths);

  subplot(2,n_arm_types,arm_type)
  for which_angle=1:n_dofs
    angles = zeros(1,n_dofs);
    for angle_sign=1:3
      angles(which_angle) = (angle_sign-2)*(pi/10);
      hold on
      x = getarmpos(angles,link_lengths,1,2);

      viapoint_count = 1;
      for viapoint_x=viapoint_xs
        for viapoint_y=viapoint_ys
          hold on
          plot(viapoint_x,viapoint_y,'.')
          viapoint = [viapoint_x viapoint_y]';
          dist_to_viapoint(which_angle,angle_sign,viapoint_count) =  sqrt(sum((x-viapoint).^2));
          viapoint_count = viapoint_count+1;
        end
      end

    end
  end
  hold off
  axis equal
  xlim([-0.1 1.1])
  ylim([-0.4 1.1])
  xlabel('x')
  ylabel('y')

  subplot(2,n_arm_types,n_arm_types+arm_type)
  cumsum_link_lengths = cumsum(link_lengths);
  cumsum_link_lengths = [0 cumsum_link_lengths(1:end-1)];
  cur_dists_plus = squeeze(dist_to_viapoint(:,1,:) - dist_to_viapoint(:,2,:));
  cur_dists_minus = squeeze(dist_to_viapoint(:,3,:) - dist_to_viapoint(:,2,:));
  for which_angle=1:n_dofs
    plot(cumsum_link_lengths(which_angle)*ones(1,2),[mean(cur_dists_minus(which_angle,:)) mean(cur_dists_plus(which_angle,:))],'LineWidth',3)
    hold on
  end
  hold off
  axis equal
  set(gca,'XTick',cumsum_link_lengths)
  set(gca,'XTickLabel',1:n_dofs)
  xlim([-0.1 1.1])
  ylim([-0.4 1.1]-0.7)
  xlabel('joint number')
  ylabel('min/max cost with 0.1\pi perturbation')

  drawnow

end

end


