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
order = 2;
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
% Initialization

% Initialize arrays
gs = zeros(T,1);
zs = zeros(T,1);
zds = zeros(T,1);
ys = zeros(T,1);
yds = zeros(T,1);
ydds = zeros(T,1);

% Initialize and integrate transformation systems
alpha_z = 14.0;
beta_z = alpha_z/4.0;
alpha_g = alpha_z/2.0;

%-------------------------------------------------------------------------------
% Each dimension integrated separately
for i_dim = 1:n_dim

  % Compute sum(phi*wi)/sum(phi)
  weighted_sum_activations = sum(activations.*repmat(theta(i_dim,:),T,1),2);
  sum_activations = sum(activations,2);
  f = (weighted_sum_activations./sum_activations).*vs;

  ys(1) = y0(i_dim);
  yds(1) = 0.0;
  ydds(1) = 0.0;
  zs(1) = 0.0;
  zds(1) = 0.0;
  gs(1) = 0.0;
  gds(1) = alpha_g*g(i_dim);
  
  %-------------------------------------------------------------------------------
  % Integration
  for tt=1:T-1
    %nonlinear = f(tt)*A;       % NONLIN_AMPLITUDE
    nonlinear = f(tt)*(g(i_dim)-y0(i_dim));  % NONLIN_GOAL
    %nonlinear = 0;             % NONLIN_ZERO
    %g_cur = gs(tt-1);          % GOAL_DISCONTINUOUS
    g_cur = g(i_dim);                 % GOAL_CONTINUOUS

    % Use one second order system (alternative: 2 first order systems)
    ydds(tt+1) = alpha_z*(beta_z*(g_cur-ys(tt))-yds(tt)) + nonlinear;

    % Integrate acceleration to get velocity and position
    yds(tt+1) = yds(tt) + dt*ydds(tt+1);
    ys(tt+1)  =  ys(tt) + dt* yds(tt+1);
    
    % Goal integration (for smooth adaptation to changing goals)
    gds(tt+1) = alpha_g*(g((i_dim))-gs(tt));
    gs(tt+1)  =  gs(tt) + dt* gds(tt+1);

  end

  trajectory.t = ts;
  trajectory.y(:,i_dim) = ys;
  trajectory.yd(:,i_dim) = yds;
  trajectory.ydd(:,i_dim) = ydds;
  
  % Plot if necessary
  if (figure_handle)
    n_rows = n_dim;
    n_cols = 8;

    subplot(n_rows,n_cols,n_cols*(i_dim-1) + 1); 
    plot(ts,activations)
    axis tight; xlabel('t (s)'); ylabel('\Psi(x)')
    title('basis functions')
    
    subplot(n_rows,n_cols,n_cols*(i_dim-1) + 2); 
    stem(theta(i_dim,:))
    axis tight; xlabel('b'); ylabel('$\theta_b$','Interpreter','LaTex')
    axis([0 n_basis_functions+1, min(min(theta)) max(max(theta))]);
    title('weights (\theta)')
        
    subplot(n_rows,n_cols,n_cols*(i_dim-1) + 3); 
    plot(ts,weighted_sum_activations./sum_activations)
    axis tight; xlabel('t (s)'); ylabel('$\frac{\sum\Psi(x)\theta}{\sum\Psi(x)}$','Interpreter','LaTex')
    title('weighted basis functions')

    subplot(n_rows,n_cols,n_cols*(i_dim-1) + 4); 
    plot(ts,vs)
    axis tight; xlabel('t (s)'); ylabel('v')
    title('canonical system (v)')
    
    subplot(n_rows,n_cols,n_cols*(i_dim-1) + 5);
    plot(ts,f)
    axis tight; xlabel('t (s)'); ylabel('f')
    title('nonlinear component (v)')

    subplot(n_rows,n_cols,n_cols*(i_dim-1) + 6);
    plot(ts,ydds)
    axis tight; xlabel('t (s)'); ylabel(['ydd_' num2str(i_dim)])
    title('acceleration')
    
    subplot(n_rows,n_cols,n_cols*(i_dim-1) + 7);
    plot(ts,ys)
    axis tight; xlabel('t (s)'); ylabel(['y_' num2str(i_dim)])
    title('position')

    if (i_dim==n_dim && n_dim>1)
      subplot(n_rows,n_cols,n_cols*((1:i_dim)-1) + 8);
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