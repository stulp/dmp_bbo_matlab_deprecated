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

init_expl = 0.1;

if (1)
  [epoch_summaries_all runpimeta_filename ] = runpimeta('icdl2012');
end

%load(runpimeta_filename)
load(sprintf('icdl2012_%1.1f.mat',init_expl))

[ n_experiments n_sessions ] = size(epoch_summaries_all);
[n_updates n_dmps n_bases ] = size(epoch_summaries_all(1,1).means);

save_svg=1;
percentile = 0.5;
%highlight_updates = round(linspace(20,150,8))%[10:10:70 150 n_updates];
highlight_updates = [5 17 26 34 50 60 150];
highlight_updates = [150 200];

plot_count = 1;
for i_experiment=1:n_experiments

  %----------------------------------------------------------
  % Get max_eigvals
  if (0)
  all_max_eigvals = zeros(n_sessions,n_updates,n_dmps);
  for session=1:n_sessions
    covars = epoch_summaries_all(i_experiment,session).covars;
    for i_update=1:n_updates
      for i_dmp=1:n_dmps
        [eigvec,eigval] = eig(squeeze(covars(i_update,i_dmp,:,:)));
        all_max_eigvals(session,i_update,i_dmp) = real(max(diag(eigval)));
        if (i_update==1)
          init_vals(i_dmp) = all_max_eigvals(session,i_update,i_dmp);
        end
        all_max_eigvals(session,i_update,i_dmp) = all_max_eigvals(session,i_update,i_dmp)-init_vals(i_dmp)+0.1;
      end
    end
  end
  end
  %max_eigvals = filter(ones(1,10)/10,1,max_eigvals);
  avg_max_eigvals = squeeze(mean(all_max_eigvals,1));
  sum_max_eigvals = squeeze(sum(all_max_eigvals,3));
  avg_sum_max_eigvals = mean(sum_max_eigvals,1);
  std_sum_max_eigvals = sqrt(var(sum_max_eigvals,0,1));

    
  %avg_max_eigvals = avg_max_eigvals(1:n_updates,:);
  base_avg_max_eigvals = avg_max_eigvals-0.1;
  cumsum_avg_max_eigvals = cumsum(base_avg_max_eigvals,2);
  norm_cumsum_avg_max_eigvals = cumsum_avg_max_eigvals./repmat(max(cumsum_avg_max_eigvals,[],2),1,n_dmps);
  norm_cumsum_avg_max_eigvals(1,:) = 0.1:0.1:1;

  n_dofs = n_dmps;
  %max_eigvals_base = avg_max_eigvals-0.1;
  %max_eigvals_base_norm = max_eigvals_base./repmat(max(max_eigvals_base,[],2),1,n_dofs);
  %max_eigvals_base_cumsum = cumsum(max_eigvals_base ,2);
  %max_eigvals_base_cumsum_norm = max_eigvals_base_cumsum./repmat(max(max_eigvals_base_cumsum,[],2),1,n_dofs);

  link_scale = 0.6;
  arm_length = 1.0;
  rel_link_lengths = link_scale.^(1:n_dofs);
  rel_link_lengths = rel_link_lengths/sum(rel_link_lengths);
  link_lengths = rel_link_lengths*arm_length;
  y = [0 cumsum(link_lengths(1:end-1))];
  %y = 1:10;% linspace(0,1,n_dofs);

  percentile_dof =zeros(1,n_updates);
  for uu=1:n_updates
    dof_index = find(norm_cumsum_avg_max_eigvals(uu,:)>percentile,1);
    if (dof_index<n_dofs)
      percentile_dof(uu) = 0.5*(y(dof_index)+y(dof_index+1));
    else
      percentile_dof(uu) = y(dof_index);
    end
  end


  %----------------------------------------------------------
  % Get mu+sigma of learning curves
  merged_curves = zeros(n_sessions,n_updates,2);
  for i_session=1:n_sessions
    merged_curves(i_session,:,:) = epoch_summaries_all(i_experiment,i_session).learning_curve;
  end

  if (sum((var(merged_curves(:,:,1))))~=0)
    warning('Learning curves have different trials/update. Averaging might not work.'); %#ok<WNTAG>
  end
  %trials = cumsum(squeeze(merged_curves(1,:,1)));
  avg_curves = squeeze(mean(merged_curves(:,:,2),1));
  std_curves = sqrt(squeeze(var(merged_curves(:,:,2),0,1)));
  %avg_min_std(avg_min_std<0.1) = 0.1;
  
  %----------------------------------------------------------
  % Goal x
  goal_x = zeros(1,n_updates);
  for i_update=1:n_updates
    load(sprintf('icdl2012_%1.1f/experiment_001/session_001/%03d_update.mat',init_expl,i_update))
    goal_x(i_update) = task.viapoint(1);
  end

  
  %----------------------------------------------------------
  % Set colormap to have 'n_dofs' entries
  colormap(ones(n_dofs,3))
  map = colormap(jet);
  %c =[ 1:ceil(n_dofs/2); (floor(n_dofs/2)+1):n_dofs]
  %map = map(c(:),:);
  map = rot90(rot90(map));
  %map(end,:) = 1;

  %----------------------------------------------------------
  % Plot learning curves
  n_highlights = length(highlight_updates);

  figure(30+plot_count); plot_count = plot_count + 1;
  %subplot(2,n_highlights,1:n_highlights)

  set(gcf,'DefaultAxesColorOrder',map)
  colormap(map);

  use_plotyy = 0;
  
  if (use_plotyy)
    subplot(4,1,1)
  else
    subplot(7,1,1)
  end    
  plot(140:260,goal_x(140:260),'-r')
  axis([1 n_updates -0.1 0.6])
  set(gca,'XTick',[]);
  
  set(gca,'YTick',0:0.25:0.5);

  avg_plus_std = avg_curves+std_curves;
  avg_min_std = avg_curves-std_curves;
  avg_plus_std_expl = avg_sum_max_eigvals+std_sum_max_eigvals;
  avg_min_std_expl = avg_sum_max_eigvals-std_sum_max_eigvals;
  

  if(use_plotyy)
    subplot(4,1,2:4)
    [AX, H(1), H(2)] = plotyy(1:n_updates,avg_curves,1:n_updates,avg_sum_max_eigvals,'semilogy','semilogy','LineWidth',2);
    hold on
    %H(3) = patch([1:n_updates n_updates:-1:1],[avg_plus_std avg_min_std(end:-1:1)],[0.7 0.7 1.0],'EdgeColor','none','Parent', AX(2));
    
    %avg_min_std(avg_min_std<0.1) = 0.1;
    %H(3) = line([1:n_updates],[avg_min_std],'Color',[0.7 0.7 1.0],'Parent', AX(1));
    %H(4) = line([1:n_updates],[avg_plus_std],'Color',[0.7 0.7 1.0],'Parent', AX(1));
    H(3) = patch([1:n_updates n_updates:-1:1],[avg_plus_std avg_min_std(end:-1:1)],[0.7 0.7 1.0],'EdgeColor','none','Parent', AX(1));
    
    
    %avg_min_std(avg_min_std<0.1) = 0.1;
    H(end+1) = line(1:n_updates,avg_min_std_expl,'Color',[0 0 0],'Parent', AX(2));
    H(end+1) = line(1:n_updates,avg_plus_std_expl,'Color',[0 0 0],'Parent', AX(2));
    
    
    set(H(1:2),'LineWidth',2)
    set(H(2),'Color',[0 0 0])
    
    uistack(H(1), 'top')
    uistack(H(2), 'top')
    hold off
  else
    set(gca,'XTick',[])
    
    
    subplot(7,1,2:4)
    semilogy(avg_curves,'LineWidth',2)
    hold on
    plot(avg_plus_std);
    plot(avg_min_std);
    hold off
    grid on
    ylim([0.5 60])
    set(gca,'XTick',[])

    if (1)
    subplot(7,1,5:7)
    plot(avg_sum_max_eigvals,'LineWidth',2)
    hold on
    plot(avg_plus_std_expl);
    plot(avg_min_std_expl);
    hold off
    grid on
    ylim([1 17])
    set(gca,'XTick',[1 20:20:n_updates])
    xlim([1 n_updates])
    end
  end
    

  %hold on
  %plot(avg_plus_std);
  %plot(avg_min_std);
  %H(2) = patch([1:n_updates n_updates:-1:1],[avg_plus_std avg_min_std(end:-1:1)]',[0.9 0.9 0.9],'EdgeColor','none');
  %uistack(H(1), 'top')
  %hold off
  %set(gca,'YScale','log')
  %set(gca,'PlotBoxAspectRatio',[1.618 1 1] )
  %axis tight
  %axis(padaxislimits([0 0 0 0.05]))

  %ylabel('cost of evaluation trial')
  %xlabel('number of trials')

  %ylim([0.7 20000])
  %set(gca,'YTick',10.^(0:5))
  %set(gca,'XTick',10:10:n_updates)

  if (save_svg)
    %colorbar
    plot2svg('svg_icdl2012_learning_curve.svg')
  end

  %----------------------------------------------------------
  % Plot some arms below the learning curve
  figure(30+plot_count); plot_count = plot_count + 1;
  for hh = 1:n_highlights
    uu = highlight_updates(hh);

    subplot(2,n_highlights,hh)
    %subplot(2,n_highlights,n_highlights+hh)
    load(sprintf('icdl2012_%1.1f/experiment_001/session_001/%03d_update.mat',init_expl,uu))
    angles = squeeze(trials(1,1).y(1,:,:));
    [ x ] = getarmpos(angles,1.0,1,2);
    hold on
    plot(task.viapoint(1),task.viapoint(2),'o','MarkerFaceColor',[1 0.3 0.3],'MarkerEdge','none','MarkerSize',10)
    hold off
    title(sprintf('update=%03d',highlight_updates(hh)));
    axis equal
    set(gca,'XTick',[])
    set(gca,'YTick',[])
    axis([-0.1 1.1 -0.1 0.75])

    subplot(2,n_highlights,n_highlights+hh)
    use_max_eigvals = avg_max_eigvals-0.1;
    %plot(y,zeros(1,n_dofs),'-k','LineWidth',2)
    for ii=1:n_dofs
      %plot(y(ii),zeros(size(y(ii))),'k.')
      %plot(uu,y(ii),'o','MarkerSize',10*max_eigvals(uu,ii))
      plot(y([ii ii]),[use_max_eigvals(uu,ii) 0],'Color',0.9*map(ii,:),'LineWidth',4)
      hold on
      %plot(y(ii)+circle_radii(uu,ii)*circle(1,:),circle_radii(uu,ii)*circle(2,:),'Color',0.9*map(ii,:),'LineWidth',2)
    end
    hold off
    %axis([-0.1 1.1 -0.1*max(max(use_max_eigvals(highlight_updates,:))) 1.1*max(max(use_max_eigvals(highlight_updates,:))) ])
    axis([-0.1 1.1 -1 4 ])
    axis square
    set(gca,'YDir','reverse')
    %xlim([1 200])
    %ylim([-0.15 1.1])
    %set(gca,'XTick',10:10:200)
    set(gca,'XTick',[])
    set(gca,'YTick',[])
    if (hh==1)
      grid on
      set(gca,'YTick',0:1:4)
    end
    %set(gca,'YTickLabel',1:10)

  end

  if (save_svg)
    plot2svg('svg_icdl2012_arm_highlights.svg')
  end

  return 
  %----------------------------------------------------------
  % Plot max_eigvals in various ways
  n_updates = highlight_updates(end);
  
  n_updates = 150;
  avg_max_eigvals = avg_max_eigvals(1:n_updates,:);
  base_avg_max_eigvals = avg_max_eigvals-0.1;
  cumsum_avg_max_eigvals = cumsum(base_avg_max_eigvals,2);
  norm_cumsum_avg_max_eigvals = cumsum_avg_max_eigvals./repmat(max(cumsum_avg_max_eigvals,[],2),1,n_dofs);
  norm_cumsum_avg_max_eigvals(1,:) = 0.1:0.1:1;
  percentile_dof = percentile_dof(1:n_updates);

  %----------------------------------------------------------
  % Raw data
  figure(30+plot_count); plot_count = plot_count + 1;
  set(gcf,'DefaultAxesColorOrder',map)
  colormap(map);

  plot(avg_max_eigvals,'LineWidth',2)
  %semilogy(avg_max_eigvals(1:150,:))
  axis tight
  axis(padaxislimits([0 0 0 0.05]))
  %ylim([0 4.2])

  if (save_svg)
    colorbar
    plot2svg('svg_icdl2012_max_eigvals.svg')
  end

  for abs_or_norm=0:1

    %----------------------------------------------------------
    % Minimum substracted, normalized and plot as patches
    figure(30+plot_count); plot_count = plot_count + 1;
    clf
    set(gcf,'DefaultAxesColorOrder',map)
    colormap(map);

    %if (abs_or_norm==0)
    %  figure(31)
    %  subplot(7,1,1); colorbar
    %  subplot(7,1,2:4); colorbar
    %  subplot(7,1,5:7)
    %end


    %subplot(2,1,1)
    if (abs_or_norm)
      patch_pad = norm_cumsum_avg_max_eigvals;
    else
      patch_pad = cumsum_avg_max_eigvals;
    end
    n = length(patch_pad);
    for ii=n_dmps:-1:1
      patch([1:n n 1 1],[ patch_pad(:,ii)' 0 0 patch_pad(1,ii)],map(ii,:),'EdgeColor','none')
      hold on
    end
    %a1 = d1./repmat(sum(d1,2),1,n_dofs);
    %best_vals = [];
    %for uu=1:n_updates
    %  best_ind = find((a1(uu,:)==max(a1(uu,:))),1);
    %  if (best_ind==1)
    %    best_vals = [best_vals (a(uu,best_ind))/2];
    %  else
    %    best_vals = [best_vals (a(uu,best_ind)+a(uu,best_ind-1))/2];
    %  end
    %end
    %plot(best_vals,'ok','LineWidth',2)
    %plot([0 n_updates],[0.8 0.8],'-k')
    if (abs_or_norm)
      plot([1 n_updates],[percentile percentile],'-k')
    end
    %plot([0 n_updates],[0.9 0.9],'-k')
    hold off
    axis tight
    colorbar
    colormap(map);


    if (save_svg)
      if (abs_or_norm)
        plot2svg('svg_icdl2012_max_eigvals_cumul_norm.svg')
      else
        plot2svg('svg_icdl2012_max_eigvals_cumul.svg')
      end
    end

    %----------------------------------------------------------
    % Arms and exploration magnitude per joint
    figure(30+plot_count); plot_count = plot_count + 1;
    %subplot(2,1,2)
    clf
    set(gcf,'DefaultAxesColorOrder',map)
    colormap(map);

    %if (abs_or_norm)
    patch([1:n_updates n_updates 1 1],[percentile_dof -0.15 -0.15 percentile_dof(1)],0.9*[1 1 1],'EdgeColor',0.7*[1 1 1])
    hold on
    %end

    aaa = linspace(0,2*pi,20);
    circle = [0.02*sin(aaa); 0.08*cos(aaa)];
    if (abs_or_norm)
      circle_radii = norm_cumsum_avg_max_eigvals;
    else
      circle_radii = 1.6*base_avg_max_eigvals/max(max(base_avg_max_eigvals));
    end
    for uu=10:10:n_updates-1
      plot(uu*ones(1,n_dofs),y,'-k','LineWidth',2)
      hold on
      for ii=1:n_dofs
        plot(uu,y(ii),'k.')
        %plot(uu,y(ii),'o','MarkerSize',10*max_eigvals(uu,ii))
        plot(uu+200*circle_radii(uu,ii)*circle(1,:),y(ii)+circle_radii(uu,ii)*circle(2,:),'Color',0.9*map(ii,:),'LineWidth',2)
      end
    end
    hold off
    xlim([1 n_updates])
    ylim([-0.15 1.1])
    set(gca,'XTick',10:10:n_updates)
    set(gca,'YTick',y)
    set(gca,'YTickLabel',1:10)
    colorbar

    if (save_svg)
      if (abs_or_norm)
        plot2svg('svg_icdl2012_max_eigvals_cumul_norm_joints.svg')
      else
        plot2svg('svg_icdl2012_max_eigvals_cumul_joints.svg')
      end
    end

  end

  %print('-dpng',sprintf('%03d_dofs.png',n_dofs));

end