function  maturationoptimizationvisualization(link_lengths_per_arm,learning_histories)

clf
n_arm_types = size(link_lengths_per_arm,1);
for arm_type=1:size(learning_histories,1)
  link_lengths = link_lengths_per_arm(arm_type,:);
  n_dofs = length(link_lengths);

  subplot(3,n_arm_types,arm_type)
  angles = 0.1*ones(1,n_dofs);
  getarmpos(angles,link_lengths,1,2);
  axis equal
  axis([-0.1 1.1 -0.1 0.6])
  
  subplot(3,n_arm_types,[n_arm_types 2*n_arm_types]+arm_type)
  title(sprintf('arm type = %d',arm_type));
  current_histories = {learning_histories{arm_type,:,:}};
  plotlearninghistorymaturation(current_histories);
end
drawnow

end
