function [ trajectory activations canonical_at_centers ] = dmpintegrate(y0,g,theta,time,dt,time_exec,order,figure_handle)
% Integrate a Dynamic Movement Primitive
%
% Input:
%   y0            - initial state
%   g             - goal state
%   theta         - DMP parameters (i.e. 'weights')
%   time          - duration of the observed movement
%   dt            - duration of the integration step
%   time_exec     - duration of the integration
%   order         - order of the canonical system (1 or 2)
%   figure_handle - figure to plot on (0: no plotting)
% Output:
%   trajectory    - the trajectory that results from integration.
%                   this is a structure that contains
%                      trajectory.t   - times
%                      trajectory.y   - position over time (for each dimension)
%                      trajectory.yd  - velocity over time (for each dimension)
%                      trajectory.ydd - acceleration over time (for each dimension)
%   activations - activations of the basis functions at each time step
%   canonical_at_centers - value of the canonical system at the centers of the
%                          basis functions

if (nargin==0)
  % If no arguments are passed, test the function
  trajectory = testdmpintegrate;
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
n_dim = size(y0,2);
if (size(g,2)~=n_dim)
  error(sprintf('Since y0 is %d X %d, g must be %d X %d, but it is %d X %d : ABORT.',size(y0),size(y0),size(g))) %#ok<SPERR>
end
% Theta is always a 1 x M vector. But DMPs require n_dim x n_basis_functions. So
% we have to reshape it here.

% Theta may have two forms
% N x M, where N=n_dim and M=n_basisfunctions
% 1 x M, where M=n_dim*n_basisfunctions
%   In the latter case, we have to reshape it into the first case
[ n m ] = size(theta);
if (n==1)
  if (mod(m,n_dim)>0)
    error(sprintf('If theta is 1 x M, then M must be a multiple of n_dim.  But %d is NOT a multiple of %d : ABORT.',m,n_dim)) %#ok<SPERR>
  else
    theta = reshape(theta,m/n_dim,n_dim)';
  end
elseif (n~=n_dim)
  error(sprintf('theta must be\n  1 X (n_dim*n_basis_functions)\nor\n  n_dim X n_basis_functions\nbut it is %d X %d and n_dim=%d : ABORT.',size(theta),n_dim)) %#ok<SPERR>
end
% All good now, theta already in correct shape for DMP
[ n_dim n_basis_functions ] = size(theta);


%-------------------------------------------------------------------------------
% Integrate canonical system in closed form
[ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>
% Duration of the motion in time steps
T = length(xs);


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
for i_dim = 1:n_dim

  % Integrate this transformation system
  if (figure_handle)
    figure_handle_per_dim = figure_handle+i_dim;
  else
    figure_handle_per_dim = 0;
  end
  trajectory_per_dim = transformationintegrate(y0(i_dim),g(i_dim),theta(i_dim,:),xs,vs,dt,figure_handle_per_dim);
 
  trajectory.t = ts;
  trajectory.y(:,i_dim)   = trajectory_per_dim.y;
  trajectory.yd(:,i_dim)  = trajectory_per_dim.yd;
  trajectory.ydd(:,i_dim) = trajectory_per_dim.ydd;
end
  
% Plot if necessary
if (figure_handle)
  
  figure(figure_handle)
  if (n_dim>1)
    if (n_dim>2)
      plot3(trajectory.y(:,1,1),trajectory.y(:,2,1),trajectory.y(:,3,1));
      zlabel('y_3');
    else
      plot(trajectory.y(:,1,1),trajectory.y(:,2,1));
    end
    xlabel('y_1'); ylabel('y_2')
    axis equal
    axis square
    axis tight;
  end
    
end


  function trajectory = testdmpintegrate
    
    % Integrate and plot a 2-D DMP with random weights
    n_basis_functions = 8;
    y0 = [0 0.1];
    g  = [1 0.9];
    theta = 10*randn(length(g),n_basis_functions);
    time = 2;
    dt = 1/100;
    time_exec = 2.5;
    order = 2;
    figure_handle = 1;
    
    trajectory = dmpintegrate(y0,g,theta,time,dt,time_exec,order,figure_handle);
    
  end

end