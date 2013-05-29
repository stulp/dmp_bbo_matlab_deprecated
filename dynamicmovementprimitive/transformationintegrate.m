function [ trajectory ] = transformationintegrate(y0,g,theta,xs,vs,dt,figure_handle)
% Integrate one transformation system of a Dynamic Movement Primitive
%
% Input:
%   y0            - initial state
%   g             - goal state
%   theta         - DMP parameters (i.e. 'weights')
%   xs            - canonical system (time signal)
%   vs            - canonical system (forcing term multiplier)
%   dt            - duration of the integration step
%   figure_handle - figure to plot on (0: no plotting)
% Output:
%   trajectory    - the trajectory that results from integration.
%                   this is a structure that contains
%                      trajectory.t   - times
%                      trajectory.y   - position over time (for each dimension)
%                      trajectory.yd  - velocity over time (for each dimension)
%                      trajectory.ydd - acceleration over time (for each dimension)

if (nargin==0)
  % If no arguments are passed, test the function
  trajectory = testtransformationintegrate;
  return;
end

%-------------------------------------------------------------------------------
% Default values
if (nargin<7), figure_handle = 0; end

n_basis_functions = length(theta);


%-------------------------------------------------------------------------------
% Duration of the motion in time steps
T = length(xs);
ts = dt*(0:T-1)'; % time over time

%-------------------------------------------------------------------------------
% Get basis function activations
centers = linspace(1,0.001,n_basis_functions);
widths = ones(size(centers))/n_basis_functions;
activations = basisfunctionactivations(centers,widths,xs);

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

% Compute sum(phi*wi)/sum(phi)
weighted_sum_activations = sum(activations.*repmat(theta,T,1),2);
sum_activations = sum(activations,2);
f = (weighted_sum_activations./sum_activations).*vs;

ys(1) = y0;
yds(1) = 0.0;
ydds(1) = 0.0;
zs(1) = 0.0;
zds(1) = 0.0;
gs(1) = 0.0;
gds(1) = alpha_g*g;

%-------------------------------------------------------------------------------
% Integration
for tt=1:T-1
  %nonlinear = f(tt)*A;       % NONLIN_AMPLITUDE
  nonlinear = f(tt)*(g-y0);  % NONLIN_GOAL
  %nonlinear = 0;             % NONLIN_ZERO
  %g_cur = gs(tt-1);          % GOAL_DISCONTINUOUS
  g_cur = g;                 % GOAL_CONTINUOUS

  % Use one second order system (alternative: 2 first order systems)
  ydds(tt+1) = alpha_z*(beta_z*(g_cur-ys(tt))-yds(tt)) + nonlinear;

  % Integrate acceleration to get velocity and position
  yds(tt+1) = yds(tt) + dt*ydds(tt+1);
  ys(tt+1)  =  ys(tt) + dt* yds(tt+1);

  % Goal integration (for smooth adaptation to changing goals)
  gds(tt+1) = alpha_g*(g-gs(tt));
  gs(tt+1)  =  gs(tt) + dt* gds(tt+1);

end

trajectory.t = ts;
trajectory.y = ys;
trajectory.yd = yds;
trajectory.ydd = ydds;

% Plot if necessary
if (figure_handle)
  figure(figure_handle)
  n_rows = 1;
  n_cols = 7;

  subplot(n_rows,n_cols,1);
  plot(ts,activations)
  axis tight; xlabel('t (s)'); ylabel('\Psi(x)')
  title('basis functions')

  subplot(n_rows,n_cols,2);
  stem(theta)
  axis tight; xlabel('b'); ylabel('$\theta_b$','Interpreter','LaTex')
  axis([0 n_basis_functions+1, min(theta) max(theta)]);
  title('weights (\theta)')

  subplot(n_rows,n_cols,3);
  plot(ts,weighted_sum_activations./sum_activations)
  axis tight; xlabel('t (s)'); ylabel('$\frac{\sum\Psi(x)\theta}{\sum\Psi(x)}$','Interpreter','LaTex')
  title('weighted basis functions')

  subplot(n_rows,n_cols,4);
  plot(ts,vs)
  axis tight; xlabel('t (s)'); ylabel('v')
  title('canonical system (v)')

  subplot(n_rows,n_cols,5);
  plot(ts,f)
  axis tight; xlabel('t (s)'); ylabel('f')
  title('nonlinear component (v)')

  subplot(n_rows,n_cols,6);
  plot(ts,ydds)
  axis tight; xlabel('t (s)'); ylabel(['ydd'])
  title('acceleration')

  subplot(n_rows,n_cols,7);
  plot(ts,ys)
  axis tight; xlabel('t (s)'); ylabel(['y'])
  title('position')

end



  function trajectory = testtransformationintegrate

    % Integrate canonical system
    dt = 1/100;
    time = 2;
    time_exec = 2.5;
    order = 2;
    [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>

    % Integrate and plot a transformation system with random weights
    n_basis_functions = 8;
    y0 = 0;
    g  = 1;
    theta = 10*randn(1,n_basis_functions);
    
    figure_handle = 1;

    trajectory = transformationintegrate(y0,g,theta,xs,vs,dt,figure_handle);

  end

end