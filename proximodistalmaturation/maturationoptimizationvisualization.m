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

function  maturationoptimizationvisualization(link_lengths_per_arm,learning_histories)

n_arm_types = size(link_lengths_per_arm,1);

subplot_handles = zeros(n_arm_types,2);
clf

arm_colors = 0.8*[1 0 0; 0 1 0; 0 0 1];

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
  
  %subplot_handles(arm_type,2) = subplot(3,n_arm_types,[n_arm_types 2*n_arm_types]+arm_type);
  subplot_handles(arm_type,2) = subplot(1,3,arm_type);
  title(sprintf('arm type = %d',arm_type));
  current_histories = {learning_histories{arm_type,:,:}};
  [max_values max_value_indices colors ] = plotlearninghistorymaturation(current_histories,arm_colors(arm_type,:));
  legend off
  
  %subplot(2,2,n_arm_types+1);
  if (0)
  [sorted_vals sorted_idx] = sort(max_value_indices); 
  plot(max_value_indices(sorted_idx),max_values(sorted_idx),'-','Color',arm_colors(arm_type,:),'LineWidth',2)
  hold on
  for ii=1:length(max_value_indices)
    plot(max_value_indices(ii),max_values(ii),'o','MarkerSize',16,'MarkerFaceColor',colors(ii,:),'MarkerEdgeColor',arm_colors(arm_type,:),'LineWidth',2)
    text(max_value_indices(ii),max_values(ii),num2str(ii),'HorizontalAlignment','center')
  end
  ylim([1/n_dofs 0.6])
  xlabel('number of updates')
  ylabel('exploration magnitude')
  set(gca,'PlotBoxAspectRatio',[1.618 1 1] );
  end
  
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
