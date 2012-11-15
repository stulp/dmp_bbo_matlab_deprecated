function [subplot_handles] = uncertaintyhandlingvisualize(link_lengths_per_arm,results)


n_arm_types = size(link_lengths_per_arm,1);

subplot_handles = zeros(n_arm_types,2);

clf
for arm_type=1:length(results)
  link_lengths = link_lengths_per_arm(arm_type,:);
  n_dofs = length(link_lengths);

  %subplot_handles(arm_type,1) = subplot(4,n_arm_types,arm_type);
  %getarmpos(0.1*ones(1,n_dofs),link_lengths,1,2);
  %axis equal
  %axis([-0.1 1.1 -0.1 0.6])
  %
  %subplot_handles(arm_type,2) = subplot(4,n_arm_types,[n_arm_types 2*n_arm_types 3*n_arm_types]+arm_type);
  
  subplot_handles(arm_type) = subplot(1,n_arm_types,arm_type);
  x = [0 cumsum(link_lengths)];
  z = results{arm_type}-0.5;
  h = bar3(x(1:end-2),z(1:end-1,:));
  %h = bar3(results{arm_type}(1:n_dofs-1,2:n_dofs));
  remove_empty_bars(h);
  %for i = 1:length(h)
  %  zdata = get(h(i),'Zdata');
  %  set(h(i),'Cdata',zdata)
  %  set(h,'EdgeColor','k')
  %end
  %set(gca,'CLim',[0 0.5])
  hold on
  plot3(1,x(end),0,'ok')
  plot3(ones(size(x)),x,zeros(size(x)),'-','Color',0.7*ones(1,3))
  x = x(1:end-1); % Last joint is not a joint, but the end-effector
  plot3(ones(size(x)),x,zeros(size(x)),'.','Color',0.3*ones(1,3),'MarkerSize',15)
  hold off
  axis equal
  axis square;
  cur_ylim = ylim;
  cur_ylim(2) = 1.1;
  ylim(cur_ylim)
  xlabel('distal joint');
  ylabel('proximal joint');
  zlabel('% that distal joint doesnt matter')
  set(gca,'YTick',x(1:end-1))
  set(gca,'YTickLabel',1:n_dofs-1)
  set(gca,'XTick',2:n_dofs)
  set(gca,'ZTick',0:0.1:0.5)
  set(gca,'ZTickLabel',0.5+(0:0.1:0.5))
  view(-45,20);
  zlim([0 0.5])
  colormap(gray)
  %n_colors = size(colormap,1);
  %map(:,1) = linspace(1.0,0.1,n_colors); % R
  %map(:,2) = linspace(0.1,1.0,n_colors); % G
  %map(:,3) = linspace(1.0,0.1,n_colors); % B
  %colormap(map)
end
drawnow

  function remove_empty_bars(hBars)
    % Taken from: http://stackoverflow.com/questions/2050367/how-to-hide-zero-values-in-bar3-plot-in-matlab
    for iSeries = 1:numel(hBars)
      zData = get(hBars(iSeries),'ZData');  %# Get the z data
      index = logical(kron(zData(2:6:end,2) == 0,ones(6,1)));  %# Find empty bars
      zData(index,:) = nan;                 %# Set the z data for empty bars to nan
      set(hBars(iSeries),'ZData',zData);    %# Update the graphics objects
    end
  end

end