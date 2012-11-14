function uncertaintyhandlingvisualize(n_dofs,arm_length,results)

clf
n_arm_types = getlinklengths;
for arm_type=1:n_arm_types
  link_lengths = getlinklengths(arm_type,n_dofs,arm_length);

  subplot(2,n_arm_types,arm_type)
  getarmpos(0.1*ones(1,n_dofs),link_lengths,1,2);
  xlim([-0.1 1.1])

  subplot(2,n_arm_types,n_arm_types+arm_type)
  h = bar3(results{arm_type});
  %h = bar3(results{arm_type}(1:n_dofs-1,2:n_dofs));
  remove_empty_bars(h);
  axis equal
  axis square;
  xlabel('distal joint');
  ylabel('proximal joint');
  zlabel('% that distal joint doesnt matter')
  set(gca,'YTick',1:n_dofs-1)
  set(gca,'XTick',2:n_dofs)
  view(-45,20);
  zlim([0 1])
  colormap(gray)
  drawnow
end

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