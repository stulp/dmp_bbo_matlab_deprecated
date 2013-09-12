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

function [ trajectory activations canonical_at_centers handle ] = dmpintegrate(y0,g,theta,time,dt,time_exec,order,figure_handle)
% Integrate a Dynamic Movement Primitive
%
% Input:
%   y0            - initial state (1 x n_trans)
%   g             - goal state (1 x n_trans)
%   theta         - DMP parameters, i.e. 'weights' (n_basis_functions x n_trans)
%   time          - duration of the observed movement
%   dt            - duration of the integration step
%   time_exec     - duration of the integration
%   order         - order of the canonical system (1 or 2)
%   figure_handle - figure to plot on (0: no plotting)
%
% Output:
%   trajectory    - the trajectory that results from integration.
%                   this is a structure that contains
%                      trajectory.t   - times (T x 1)
%                      trajectory.y   - position over time (T x n_trans)
%                      trajectory.yd  - velocity over time (T x n_trans)
%                      trajectory.ydd - acceleration over time (T x n_trans)
%   activations - activations of the basis functions at each time step
%   canonical_at_centers - value of the canonical system at the centers of the
%                          basis functions
%   handle - handle to the graph that is plotted if figure_handle>0

if (nargin==0)
  % If no arguments are passed, test the function
  [ trajectory activations canonical_at_centers handle ] = testdmpintegrate;
  return;
end

%-------------------------------------------------------------------------------
% Default values
if (nargin<5), dt = 1/100; end
if (nargin<6), time_exec = time; end
if (nargin<7), order  = 2; end
if (nargin<8), figure_handle = 0; end

%-------------------------------------------------------------------------------
% Check for consistency of dimensions, etc.
n_trans = size(y0,2);
if (size(g,2)~=n_trans)
  error(sprintf('Since y0 is %d X %d, g must be %d X %d, but it is %d X %d : ABORT.',size(y0),size(y0),size(g))) %#ok<SPERR>
end
% Theta is always a 1 x M vector. But DMPs require n_trans x n_basis_functions. So
% we have to reshape it here.

% Theta may have two forms
% N x M, where N=n_trans and M=n_basisfunctions
% 1 x M, where M=n_trans*n_basisfunctions
%   In the latter case, we have to reshape it into the first case
[ n m ] = size(theta);
if (n==1)
  if (mod(m,n_trans)>0)
    error(sprintf('If theta is 1 x M, then M must be a multiple of n_trans.  But %d is NOT a multiple of %d : ABORT.',m,n_trans)) %#ok<SPERR>
  else
    theta = reshape(theta,m/n_trans,n_trans)';
  end
elseif (n~=n_trans)
  error(sprintf('theta must be\n  1 X (n_trans*n_basis_functions)\nor\n  n_trans X n_basis_functions\nbut it is %d X %d and n_trans=%d : ABORT.',size(theta),n_trans)) %#ok<SPERR>
end
% All good now, theta already in correct shape for DMP
[ n_trans n_basis_functions ] = size(theta);


%-------------------------------------------------------------------------------
% Integrate canonical system in closed form
[ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>


%-------------------------------------------------------------------------------
% Get basis function activations
centers = linspace(1,0.001,n_basis_functions);
widths = ones(size(centers))/n_basis_functions;
activations = basisfunctionactivations(centers,widths,xs);

% Get indices when basis functions are max, i.e. at theirs centers
[ max_activations max_activation_indices ] = max(activations);
% Value of canonical system at the centers of the basis functions
canonical_at_centers = vs(max_activation_indices)';

%-------------------------------------------------------------------------------
% Each dimension integrated separately
for i_trans = 1:n_trans

  % Integrate this transformation system
  if (figure_handle)
    figure_handle_per_trans = figure_handle+i_trans-1;
  else
    figure_handle_per_trans = 0;
  end
  trajectory_per_trans = transformationintegrate(y0(i_trans),g(i_trans),theta(i_trans,:),xs,vs,dt,time,figure_handle_per_trans);

  trajectory.t = ts;
  trajectory.y(:,i_trans)   = trajectory_per_trans.y;
  trajectory.yd(:,i_trans)  = trajectory_per_trans.yd;
  trajectory.ydd(:,i_trans) = trajectory_per_trans.ydd;

  if (figure_handle)
    set(gcf,'Name',sprintf('Transformation System %d',i_trans));
  end

end

% Plot if necessary
handle = [];
if (figure_handle)
  figure(figure_handle+n_trans)
  clf

  if (n_trans>3)
    for i_trans=1:n_trans
      subplot(1,n_trans,i_trans)
      plot(trajectory.t(1),trajectory.y(1,i_trans,1),'o','MarkerFaceColor','r','MarkerEdgeColor','none');
      hold on
      plot(trajectory.t(end),trajectory.y(end,i_trans,1),'o','MarkerFaceColor','g','MarkerEdgeColor','none');
      handle(end+1) = plot(trajectory.t,trajectory.y(:,i_trans,1));
      hold off
      xlabel('t (s)'); ylabel('y');
      axis equal
      axis square
      axis tight;
    end
  else
    if (n_trans==2)
      plot(trajectory.y(1,1,1),trajectory.y(1,2,1),'o','MarkerFaceColor','r','MarkerEdgeColor','none');
      hold on
      plot(trajectory.y(end,1,1),trajectory.y(end,2,1),'o','MarkerFaceColor','g','MarkerEdgeColor','none');
      handle = plot(trajectory.y(:,1,1),trajectory.y(:,2,1));
      hold off
      xlabel('y_1'); ylabel('y_2'); zlabel('y_3');
    elseif (n_trans==3)
      plot3(trajectory.y(1,1,1),trajectory.y(1,2,1),trajectory.y(1,3,1),'o','MarkerFaceColor','r','MarkerEdgeColor','none');
      hold on
      plot3(trajectory.y(end,1,1),trajectory.y(end,2,1),trajectory.y(end,3,1),'o','MarkerFaceColor','g','MarkerEdgeColor','none');
      handle = plot3(trajectory.y(:,1,1),trajectory.y(:,2,1),trajectory.y(:,3,1));
      xlabel('y_1'); ylabel('y_2'); zlabel('y_3');
    end
    legend('y_0','g_0','Location','EastOutside')
    axis equal
    axis square
    axis tight;
  end

  set(handle,'LineWidth',2);
  set(handle,'Color',[0.4 0.4 0.8]);

  set(gcf,'Name',sprintf('%dD DMP',n_trans));

end

  function [ trajectory activations canonical_at_centers handle ] = testdmpintegrate

    % Integrate and plot a 2-D DMP with random weights
    n_basis_functions = 8;
    y0 = [ 1.3 0.1 1 3];
    g  = [-2.0 0.9 2 2];
    n_trans = length(g);
    theta = 25*randn(n_trans,n_basis_functions);
    time = 2;
    dt = 1/100;
    time_exec = 3;
    order = 3;

    figure_handle = 1;
    [ trajectory activations canonical_at_centers handle ] = dmpintegrate(y0,g,theta,time,dt,time_exec,order,figure_handle);

  end

end