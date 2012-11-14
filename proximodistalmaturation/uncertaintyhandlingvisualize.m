function uncertaintyhandlingvisualize(link_lengths_per_arm,results)

clf
n_arm_types = size(link_lengths_per_arm,1);
for arm_type=1:length(results)
  link_lengths = link_lengths_per_arm(arm_type,:);
  n_dofs = length(link_lengths);

  subplot(3,n_arm_types,arm_type)
  getarmpos(0.1*ones(1,n_dofs),link_lengths,1,2);
  axis equal
  axis([-0.1 1.1 -0.1 0.6])

  subplot(3,n_arm_types,[n_arm_types 2*n_arm_types]+arm_type)
  h = bar3(results{arm_type}-0.5);
  %h = bar3(results{arm_type}(1:n_dofs-1,2:n_dofs));
  remove_empty_bars(h);
  axis equal
  axis square;
  xlabel('distal joint');
  ylabel('proximal joint');
  zlabel('% that distal joint doesnt matter')
  set(gca,'YTick',1:n_dofs-1)
  set(gca,'XTick',2:n_dofs)
  set(gca,'ZTick',0:0.1:0.5)
  set(gca,'ZTickLabel',0.5+(0:0.1:0.5))
  view(-45,20);
  zlim([0 0.5])
  colormap(gray)
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