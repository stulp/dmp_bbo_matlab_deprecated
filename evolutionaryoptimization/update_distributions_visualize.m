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

function subplot_handles = update_distributions_visualize(update_summary,highlight,plot_samples,i_dofs,main_color)
if (nargin<2), highlight=0; end
if (nargin<3), plot_samples=0; end
if (nargin<5), main_color=0.8*ones(1,3); end

n_dofs = length(update_summary.distributions);
n_dims  = length(update_summary.distributions(1).mean);
plot_n_dim = min(n_dims,3); % Plot only first three dimensions

weights = update_summary.weights;
n_samples = length(weights);

if (nargin<4), i_dofs=1:n_dofs; end

marker_scale = 40;

for i_dof=i_dofs
  theta = update_summary.distributions(i_dof).mean;
  covar = update_summary.distributions(i_dof).covar;
  theta_new = update_summary.distributions_new(i_dof).mean;
  covar_new = update_summary.distributions_new(i_dof).covar;
  samples = squeeze(update_summary.samples(i_dof,:,:));
  weights_normalized = weights/max(weights);
  
  if (length(i_dofs)>1)
    subplot_handles(i_dof) = subplot(1,n_dofs,i_dof);
  end
  axis equal
  if (plot_samples)
    for k=1:n_samples
      theta_k = samples(k,:);
      marker_size = round(marker_scale*weights_normalized(k))+3;
      if (plot_n_dim==2)
        % Green circle representing weight
        %patch(theta_k(1)+weights_normalized(k,1)*circle(:,1),theta_k(2)+weights_normalized(k,2)*circle(:,2),[0.7 1 0.7],'EdgeColor','none')
        plot(theta_k(1),theta_k(2),'o','MarkerFaceColor',[0.7 1 0.7],'MarkerEdgeColor','none','MarkerSize',marker_size);
        hold on
        % Line from current mean to theta_k
        plot([theta(1) theta_k(1)],[theta(2) theta_k(2)],'-','Color',[0.5 0.5 1.0])
        % theta_k
        plot(theta_k(1),theta_k(2),'o','MarkerFaceColor',[0.5 0.5 1.0],'MarkerEdgeColor','k')
      elseif (plot_n_dim==3)
        %warning('Cannot plot green circle representing weight in 3 dimensions') %#ok<WNTAG>
        % Line from current mean to theta_k
        plot3([theta(1) theta_k(1)],[theta(2) theta_k(2)],[theta(3) theta_k(3)],'-','Color',[0.5 0.5 1.0])
        hold on
        % theta_k
        plot3(theta_k(1),theta_k(2),theta_k(3),'o','MarkerFaceColor',[0.5 0.5 1.0],'MarkerEdgeColor','k')
      end

    end
    % Line from current to new theta
    plot([theta(1) theta_new(1)],[theta(2) theta_new(2)],'-','Color',0*[0.5 0 1],'LineWidth',3)
  end
  
  if (plot_n_dim==2)
    h_before_theta = plot(theta(1),    theta(2)    ,'o','MarkerSize',8,'MarkerEdgeColor','none');
    hold on
    h_after_theta  = plot(theta_new(1),theta_new(2),'o','MarkerSize',8,'MarkerEdgeColor','none');
  elseif (plot_n_dim==3)
    h_before_theta = plot3(theta(1),    theta(2)    ,    theta(3),'o','MarkerSize',8,'MarkerEdgeColor','none');
    hold on
    h_after_theta  = plot3(theta_new(1),theta_new(2),theta_new(3),'o','MarkerSize',8,'MarkerEdgeColor','none');
  end
    
  % Note that plotting this might lead to numerical issues because we are
  % taking a submatrix of the covariance matrix
  h_before_covar = error_ellipse(real(squeeze(covar(1:plot_n_dim,1:plot_n_dim))),theta(1:plot_n_dim));
  h_after_covar  = error_ellipse(real(squeeze(covar_new(1:plot_n_dim,1:plot_n_dim))),theta_new(1:plot_n_dim));

  if (highlight)
    set([h_before_covar h_after_covar],'LineWidth',2)
    set(h_before_theta ,'MarkerFaceColor',[0.2 0.2 0.7],'MarkerEdgeColor','k')
    set(h_after_theta ,'MarkerFaceColor',[0.9 0.2 0.2],'MarkerEdgeColor','k')
    if (plot_n_dim<3)
      set(h_before_covar,'Color',[0.2 0.2 0.7]);
      set(h_after_covar,'Color',[0.9 0.2 0.2]);
    end
  else
    if (plot_n_dim<3)
      set([h_before_covar h_after_covar],'Color',main_color,'LineWidth',1);
      set(h_before_theta ,'MarkerFaceColor',main_color,'MarkerEdgeColor','none');
    end
  end
end
