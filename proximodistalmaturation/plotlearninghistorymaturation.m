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

function [max_values max_value_indices colors exploration_curves] = plotlearninghistorymaturation(learning_histories,arm_color,exploration_curves)
if (isstruct(learning_histories))
  % You can pass either one learning history (a struct), or a cell array of
  % learning histories. In the former case, convert the struct to a cell
  % array with one entry.
  tmp{1} = learning_histories;
  learning_histories = tmp;
end

n_histories = length(learning_histories);
n_updates = length(learning_histories{1});
if (n_updates==1)
  % Not much progress to plot with only one update
  return
end
n_dofs = length(learning_histories{1}(1).distributions_new);

if (nargin<3)
exploration_curves = zeros(n_histories,n_updates,n_dofs);
for i_history=1:n_histories %#ok<FXUP>
  learning_history = learning_histories{i_history};
  for i_dof=1:n_dofs %#ok<FXUP>
    for hh=1:length(learning_history)
      covar = learning_history(hh).distributions(i_dof).covar;
      exploration_curves(i_history,hh,i_dof) = real(max(eig(covar))); % HACK
    end
  end
end

% Take mean over all experiments
exploration_curves = squeeze(mean(exploration_curves,1));
% Compute cumulative exploration magnitudes
cumsum_exploration_curves = cumsum(exploration_curves,2); %#ok<NASGU>
end

%----------------------------------------------------------
% Start plotting things
for ii=1:n_dofs
  labels{ii} = sprintf('joint %d',ii);
end

if (n_updates==1)
  clf
end

%----------------------------------------------------------
% Set colormap to have 'n_dofs' entries
colormap(ones(n_dofs,3))
map = colormap(jet);
map = rot90(rot90(map));
colors = map;

map_hsv = rgb2hsv(map);
hsv_map_new = map_hsv; 
hsv_map_new(:,2) = 0.3; 
colors_faded = hsv2rgb(hsv_map_new);

%map = repmat(linspace(0.3,1.0,n_dofs)',3);
colormap(map);
%c =[ 1:ceil(n_dofs/2); (floor(n_dofs/2)+1):n_dofs]
%map = map(c(:),:);
%map(end,:) = 1;
set(gcf,'DefaultAxesColorOrder',map)
colormap(map);


%----------------------------------------------------------
% Raw data
%subplot(1,2,1)
plot(exploration_curves,'LineWidth',2)
if (n_updates>1)
  legend(labels,'Location','NorthEast')
end
axis tight

%----------------------------------------------------------
% Normalized and plot as patches

if (1)
  cla
  %subplot(1,2,2)
  patch_pad = exploration_curves; % cumsum_exploration_curves;
  %[max_values max_value_indices] = max(exploration_curves)
  for tt=1:size(patch_pad,1)
    patch_pad(tt,:) = patch_pad(tt,:)./sum(patch_pad(tt,:));
  end
  [max_values max_value_indices] = max(patch_pad);
  patch_pad = cumsum(patch_pad,2);
  for ii=n_dofs:-1:1
    patch([1:n_updates n_updates 1 1],[ patch_pad(:,ii)' 0 0 patch_pad(1,ii)],colors_faded(ii,:),'EdgeColor',0.4*[1 1 1])
    hold on
  end

  for ii=1:n_dofs
    if (ii==1)
      lower_value = 0;
    else
      lower_value = patch_pad(max_value_indices(ii),ii-1);
    end
    upper_value = patch_pad(max_value_indices(ii),ii);
    


    plot(max_value_indices([ii ii]),[lower_value upper_value],'-k','LineWidth',3)
    text(max_value_indices(ii),mean([lower_value upper_value]),sprintf('%1.2f',max_values(ii)),...
      'FontSize',12,'Rotation',0,'HorizontalAlignment','left','VerticalAlignment','top');

    plot(max_value_indices(ii),mean([lower_value upper_value]),'o','MarkerSize',16,'MarkerFaceColor',colors(ii,:),'MarkerEdgeColor',[0 0 0],'LineWidth',2)
    text(max_value_indices(ii),mean([lower_value upper_value]),num2str(ii),'HorizontalAlignment','center')

  end

  sum_exploration_curves = sum(exploration_curves,2);
  plot(sum_exploration_curves/13,'-y','LineWidth',5)
  plot(sum_exploration_curves/13,'-k','LineWidth',2)
  hold off
  legend(labels{end:-1:1},'Location','EastOutside')
  axis tight
  %title('Exploration magnitude over time')
  xlabel('number of updates')
  ylabel('exploration magnitude')
  ylim([0 1])
end
set(gca,'PlotBoxAspectRatio',[1.618 1 1] );

drawnow
end
