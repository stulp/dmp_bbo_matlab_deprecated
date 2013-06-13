function [ trajectory handles_lines] = transformationintegrate(y0,g,theta,xs,vs,dt,figure_handle)
% Integrate one transformation system of a Dynamic Movement Primitive
%
% Input:
%   y0            - initial state (1 x 1)
%   g             - goal state (1 x 1)
%   theta         - DMP parameters, i.e. 'weights' (1 x n_basis_functions)
%   xs            - canonical system (time signal)
%   vs            - canonical system (forcing term multiplier)
%   dt            - duration of the integration step
%   figure_handle - figure to plot on (0: no plotting)
% Output:
%   trajectory    - the trajectory that results from integration.
%                   this is a structure that contains
%                      trajectory.t   - times ( N x 1 )
%                      trajectory.y   - position over time ( N x 1 ) 
%                      trajectory.yd  - velocity over time ( N x 1 )
%                      trajectory.ydd - acceleration over time ( N x 1 )
%    handles_lines - handles to the line objects, so that you may change their
%                    color and style in the script calling this function

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

%-------------------------------------------------------------------------------
% Get basis function activations
centers = linspace(1,0.001,n_basis_functions);
widths = ones(size(centers))/n_basis_functions;
activations = basisfunctionactivations(centers,widths,xs);

%-------------------------------------------------------------------------------
% Initialization

% Initialize arrays
gs = zeros(T,1);
%zs = zeros(T,1);
%zds = zeros(T,1);
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

% Initial conditions
tt = 1;
ys(tt) = y0;
yds(tt) = 0.0;
nonlinear = f(tt)*(g-y0);  % NONLIN_GOAL
ydds(tt) = alpha_z*(beta_z*(g-ys(tt))-yds(tt)) + nonlinear;
%zs(tt) = 0.0;
%zds(tt) = 0.0;
gs(tt) = 0.0;
gds(tt) = alpha_g*g;

%-------------------------------------------------------------------------------
% Integration
for tt=2:T
  %nonlinear = f(tt)*A;       % NONLIN_AMPLITUDE
  nonlinear = f(tt)*(g-y0);  % NONLIN_GOAL
  %nonlinear = 0;             % NONLIN_ZERO
  %g_cur = gs(tt-1);          % GOAL_DISCONTINUOUS
  g_cur = g;                 % GOAL_CONTINUOUS

  % Use one second order system (alternative: 2 first order systems)
  ydds(tt) = alpha_z*(beta_z*(g_cur-ys(tt-1))-yds(tt-1)) + nonlinear;

  % Integrate acceleration to get velocity and position
  yds(tt) = yds(tt-1) + dt*ydds(tt);
  ys(tt)  =  ys(tt-1) + dt* yds(tt);

  % Goal integration (for smooth adaptation to changing goals)
  gds(tt) = alpha_g*(g-gs(tt-1));
  gs(tt)  =  gs(tt-1) + dt*gds(tt);

end

ts = dt*(0:T-1)'; % time over time
trajectory.t = ts;
trajectory.y = ys;
trajectory.yd = yds;
trajectory.ydd = ydds;

% Plot if necessary
handles_lines = [];
if (figure_handle)
  figure(figure_handle)
  clf
  n_rows = 3;
  n_cols = 4;

  
  %----------------------------------------------------
  % First row: show spring-damper system only (i.e. without the forcing term)
  [ trajectory_no_theta ] = transformationintegrate(y0,g,zeros(size(theta)),xs,vs,dt);

  subplot(n_rows,n_cols,1+0*n_cols);
  handles_lines(end+1) = plot(ts,trajectory_no_theta.ydd);
  axis tight; xlabel('t (s)'); ylabel('ydd')
  title('acceleration (w/o forcing term)')

  subplot(n_rows,n_cols,1+1*n_cols);
  handles_lines(end+1) = plot(ts,trajectory_no_theta.yd);
  axis tight; xlabel('t (s)'); ylabel('yd')
  title('velocity (w/o forcing term)')

  subplot(n_rows,n_cols,1+2*n_cols);
  handles_lines(end+1) = plot(ts,trajectory_no_theta.y);
  axis tight; xlabel('t (s)'); ylabel('y')
  title('position (w/o forcing term)')

  
  %----------------------------------------------------
  % Second row: basis functions
  subplot(n_rows,n_cols,2+0*n_cols);
  plot(ts,activations);
  axis tight; xlabel('t (s)'); ylabel('\Psi(x)')
  title('basis functions')

  subplot(n_rows,n_cols,2+1*n_cols);
  handles_lines(end+1) = stem(theta);
  axis tight; xlabel('b'); ylabel('$\theta_b$','Interpreter','LaTex')
  axis([0.5 n_basis_functions+0.5, min(theta) max(theta)]);
  set(gca,'XTick',1:n_basis_functions)
  title('weights (\theta)')

  subplot(n_rows,n_cols,2+2*n_cols);
  handles_lines(end+1) = plot(ts,weighted_sum_activations./sum_activations);
  axis tight; xlabel('t (s)'); ylabel('$\frac{\sum\Psi(x)\theta}{\sum\Psi(x)}$','Interpreter','LaTex')
  title('weighted basis functions')

  %----------------------------------------------------
  % Third row: forcing term
  subplot(n_rows,n_cols,3+0*n_cols);
  handles_lines(end+1) = plot(ts,vs);
  axis tight; xlabel('t (s)'); ylabel('v')
  title('canonical system (v)')

  subplot(n_rows,n_cols,3+1*n_cols);
  handles_lines(end+1) = plot(ts,f);
  axis tight; xlabel('t (s)'); ylabel('f')
  title('nonlinear component (f)')

  %----------------------------------------------------
  % Fourth row: output of DMP
  subplot(n_rows,n_cols,4+0*n_cols);
  handles_lines(end+1) = plot(ts,ydds);
  axis tight; xlabel('t (s)'); ylabel('ydd')
  title('acceleration')

  subplot(n_rows,n_cols,4+1*n_cols);
  handles_lines(end+1) = plot(ts,yds);
  axis tight; xlabel('t (s)'); ylabel('yd')
  title('velocity')

  subplot(n_rows,n_cols,4+2*n_cols);
  handles_lines(end+1) = plot(ts,ys);
  axis tight; xlabel('t (s)'); ylabel('y')
  title('position')

  set(handles_lines,'LineWidth',2);
  set(handles_lines,'Color',[0.4 0.4 0.8]);
  
end



  function trajectory = testtransformationintegrate

    % Integrate canonical system
    dt = 1/100;
    time = 2;
    time_exec = 2.5;
    order = 1;
    [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>

    % Integrate and plot a transformation system with random weights
    n_basis_functions = 8;
    y0 = 0;
    g  = 1;
    theta = 250*randn(1,n_basis_functions);
    
    figure_handle = 1;

    trajectory = transformationintegrate(y0,g,theta,xs,vs,dt,figure_handle);

  end

end