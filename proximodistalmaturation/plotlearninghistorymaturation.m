function [exploration_curves] = plotlearninghistorymaturation(learning_histories,exploration_curves)
if (isstruct(learning_histories))
  % You can pass either one learning history (a struct), or a cell array of
  % learning histories. In the former case, convert the struct to a cell
  % array with one entry.
  tmp{1} = learning_histories;
  learning_histories = tmp;
end

n_histories = length(learning_histories);
n_updates = length(learning_histories{1});
n_dofs = size(learning_histories{1}(1).theta,1);

if (nargin<2)
exploration_curves = zeros(n_histories,n_updates,n_dofs);
for i_history=1:n_histories %#ok<FXUP>
  learning_history = learning_histories{i_history};
  for i_dof=1:n_dofs %#ok<FXUP>
    for hh=1:length(learning_history)
      exploration_curves(i_history,hh,i_dof) = real(max(eig(squeeze(learning_history(hh).covar(i_dof,:,:))))); % HACK
    end
  end
end

% Take mean over all experiments
exploration_curves = squeeze(mean(exploration_curves,1));
% Compute cumulative exploration magnitudes
cumsum_exploration_curves = cumsum(exploration_curves,2);
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
%c =[ 1:ceil(n_dofs/2); (floor(n_dofs/2)+1):n_dofs]
%map = map(c(:),:);
map = rot90(rot90(map));
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
    patch([1:n_updates n_updates 1 1],[ patch_pad(:,ii)' 0 0 patch_pad(1,ii)],map(ii,:),'EdgeColor',0*[1 1 1])
    hold on
    plot(max_value_indices([ii ii]),[0 patch_pad(max_value_indices(ii),ii)],'-k','LineWidth',3)
  end
  sum_exploration_curves = sum(exploration_curves,2);
  plot(sum_exploration_curves/13,'-k','LineWidth',5)
  plot(sum_exploration_curves/13,'-w','LineWidth',2)
  hold off
  legend(labels{end:-1:1},'Location','EastOutside')
  axis tight
  title('Exploration magnitude over time')
  xlabel('number of updates')
  ylabel('exploration magnitude')
  ylim([0 1])
end

drawnow
end
