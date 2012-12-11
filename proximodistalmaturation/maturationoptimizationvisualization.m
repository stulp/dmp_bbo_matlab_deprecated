function  maturationoptimizationvisualization(link_lengths_per_arm,learning_histories)

n_arm_types = size(link_lengths_per_arm,1);

subplot_handles = zeros(n_arm_types,2);
clf
for arm_type=1:size(learning_histories,1)
  link_lengths = link_lengths_per_arm(arm_type,:);
  n_dofs = length(link_lengths);

  if (0)
  subplot_handles(arm_type,1) = subplot(3,n_arm_types,arm_type);
  angles = 0.0*ones(1,n_dofs);
  getarmpos(angles,link_lengths,1,2);
  hold on
  colormap(ones(n_dofs,3))
  map = colormap(jet);
  map = rot90(rot90(map));
  joint_pos = [0 cumsum(link_lengths)];
  for dd=1:n_dofs
    plot(joint_pos([dd dd]),[-0.02 -0.1],'Color',map(dd,:),'LineWidth',2)
  end
  hold off
  
  axis equal
  axis([-0.3 1.3 -0.1 0.1])
  axis off
  end
  
  subplot_handles(arm_type,2) = subplot(3,n_arm_types,[n_arm_types 2*n_arm_types]+arm_type);
  title(sprintf('arm type = %d',arm_type));
  current_histories = {learning_histories{arm_type,:,:}};
  plotlearninghistorymaturation(current_histories);
  legend off

  if (0)
  pos_subplot_1 = get(subplot_handles(arm_type,1),'Position');
  pos_subplot_2 = get(subplot_handles(arm_type,2),'Position');
  pos_subplot_1(2) = pos_subplot_2(2) + 0.75*pos_subplot_2(4);
  set(subplot_handles(arm_type,1),'Position',pos_subplot_1);
  %set(subplot_handle_2,'Position',pos_subplot_2);
  end
end

%max_y = 0;
%for sp=1:arm_type
%  cur_ylim = get(subplot_handles(sp,2),'YLim');
%  max_y = max(max_y,cur_ylim(2));
%end
%set(subplot_handles(:,2),'YLim',[0 1.15*max_y]);


drawnow

end
