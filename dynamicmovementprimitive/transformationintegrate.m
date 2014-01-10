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

function [ trajectory handles_lines] = transformationintegrate(y0,g0,theta,xs,vs,dt,time,figure_handle)
% Integrate one transformation system of a Dynamic Movement Primitive
%
% Input:
%   y0            - initial state (1 x 1)
%   g0            - goal state (1 x 1)
%   theta         - DMP parameters, i.e. 'weights' (1 x n_basis_functions)
%   xs            - canonical system (time signal)
%   vs            - canonical system (forcing term multiplier)
%   dt            - duration of the integration step
%   figure_handle - figure to plot on (0: no plotting)
%
% Output:
%   trajectory    - the trajectory that results from integration.
%                   this is a structure that contains
%                      trajectory.t   - times ( N x 1 )
%                      trajectory.y   - position over time ( N x 1 )
%                      trajectory.yd  - velocity over time ( N x 1 )
%                      trajectory.ydd - acceleration over time ( N x 1 )
%   handles_lines - handles to the line objects, so that you may change their
%                   color and style in the script calling this function

if (nargin==0)
  % If no arguments are passed, test the function
  trajectory = testtransformationintegrate;
  return;
end


%-------------------------------------------------------------------------------
% Default values
if (nargin<8), figure_handle = 0; end

n_basis_functions = length(theta);


%-------------------------------------------------------------------------------
% Duration of the motion in time steps
T = length(xs);
ts = dt*(0:T-1)'; % time over time

%-------------------------------------------------------------------------------
% Compute basis function activations
time_instead_of_phase=1;
if (time_instead_of_phase)
  % Time signal is time
  ps = ts;
  % Get centers and widths
  [centers widths] = basisfunctioncenters(n_basis_functions,time);
else
  % Time signal is phase
  ps = xs;
  % Reconstruct alpha
  alpha = -time*log(xs(2))/dt;
  % Get centers and widths
  [centers widths] = basisfunctioncenters(n_basis_functions,time,alpha);
end

% Compute activations
activations = basisfunctionactivations(centers,widths,ps);

%-------------------------------------------------------------------------------
% Initialization

% Initialize arrays
gs = zeros(T,1);
gds = zeros(T,1);
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

gds(tt) = alpha_g*(g0-y0)/time;
gs(tt) = y0; %dt*rds(tt);

ys(tt) = y0;
yds(tt) = 0.0;
nonlinear = f(tt)*(g0-y0);  % NONLIN_GOAL
ydds(tt) = (alpha_z*(beta_z*(gs(tt)-ys(tt))-yds(tt)) + nonlinear)/time;

%-------------------------------------------------------------------------------
% Integration
for tt=2:T
  %nonlinear = f(tt)*A;       % NONLIN_AMPLITUDE
  nonlinear = f(tt)*(g0-y0);  % NONLIN_GOAL
  %nonlinear = 0;             % NONLIN_ZERO
  g_cur = gs(tt-1);          % GOAL_CONTINUOUS
  %g_cur = g0;                 % GOAL_DISCONTINUOUS

  % Use one second order system (alternative: 2 first order systems)
  ydds(tt) = (alpha_z*(beta_z*(g_cur-ys(tt-1))-yds(tt-1)) + nonlinear)/time;

  % Integrate acceleration to get velocity and position
  yds(tt) = yds(tt-1) + dt*ydds(tt);
  ys(tt)  =  ys(tt-1) + dt* yds(tt);

  % Goal integration (for smooth adaptation to changing goals)
  gds(tt) = alpha_g*(g0-gs(tt-1))/time;
  gs(tt)  =  gs(tt-1) + dt*gds(tt);

end

trajectory.t = ts;
trajectory.y = ys;
trajectory.yd = yds;
trajectory.ydd = ydds;

% Plot if necessary
handles_lines = [];
if (figure_handle)
  figure(figure_handle)
  clf
  set(gcf,'Name','Transformation system integration');

  n_rows = 3;
  n_cols = 5;

  %----------------------------------------------------
  % First column: show goal system
  subplot(n_rows,n_cols,1+1*n_cols);
  handles_lines(end+1) = plot(ts,gds);
  axis tight; xlabel('t (s)');   ylabel('$\dot{g}$','Interpreter','LaTex')
  title('$\tau\dot{g} = \alpha_g*(g_0-g)$','Interpreter','LaTex')

  subplot(n_rows,n_cols,1+2*n_cols);
  handles_lines(end+1) = plot(ts,gs);
  axis tight; xlabel('t (s)');   ylabel('$g$','Interpreter','LaTex')
  title('delayed goal')

  %----------------------------------------------------
  % Second column: show spring-damper system only (i.e. without the forcing term)
  [ trajectory_no_theta ] = transformationintegrate(y0,g0,zeros(size(theta)),xs,vs,dt,time);

  subplot(n_rows,n_cols,2+0*n_cols);
  handles_lines(end+1) = plot(ts,trajectory_no_theta.ydd);
  axis tight; xlabel('t (s)'); ylabel('$\ddot{y}$','Interpreter','LaTex')
  title('$\tau\ddot{y} = (\alpha_z*(\beta_z*(g-y)-\dot{y}))$','Interpreter','LaTex');

  subplot(n_rows,n_cols,2+1*n_cols);
  handles_lines(end+1) = plot(ts,trajectory_no_theta.yd);
  axis tight; xlabel('t (s)'); ylabel('$\dot{y}$','Interpreter','LaTex')
  title('velocity (w/o forcing term)')

  subplot(n_rows,n_cols,2+2*n_cols);
  handles_lines(end+1) = plot(ts,trajectory_no_theta.y);
  axis tight; xlabel('t (s)'); ylabel('$y$','Interpreter','LaTex')
  title('position (w/o forcing term)')


  %----------------------------------------------------
  % Third column: basis functions
  subplot(n_rows,n_cols,3+0*n_cols);
  plot(ts,activations);
  axis tight; xlabel('t (s)'); ylabel('\Psi(x)')
  title('basis functions')

  subplot(n_rows,n_cols,3+1*n_cols);
  handles_lines(end+1) = stem(theta);
  axis tight; xlabel('b'); ylabel('$\theta_b$','Interpreter','LaTex')
  axis([0.5 n_basis_functions+0.5, min(theta) max(theta)]);
  set(gca,'XTick',1:n_basis_functions)
  title('weights (\theta)')

  subplot(n_rows,n_cols,3+2*n_cols);
  handles_lines(end+1) = plot(ts,weighted_sum_activations./sum_activations);
  axis tight; xlabel('t (s)'); ylabel('$\frac{\sum\Psi(x)\theta}{\sum\Psi(x)}$','Interpreter','LaTex')
  title('weighted basis functions')

  %----------------------------------------------------
  % Fourth column: forcing term
  subplot(n_rows,n_cols,4+0*n_cols);
  handles_lines(end+1) = plot(ts,vs);
  axis tight; xlabel('t (s)'); ylabel('v')
  title('canonical system (v)')

  subplot(n_rows,n_cols,4+1*n_cols);
  handles_lines(end+1) = plot(ts,f);
  axis tight; xlabel('t (s)'); ylabel('$\frac{\sum\Psi(x)\theta}{\sum\Psi(x)}v$','Interpreter','LaTex')
  title('nonlinear component (f)')

  %----------------------------------------------------
  % Fifth column: output of DMP
  subplot(n_rows,n_cols,5+0*n_cols);
  handles_lines(end+1) = plot(ts,ydds);
  axis tight; xlabel('t (s)'); ylabel('ydd')
  title('acceleration')

  subplot(n_rows,n_cols,5+1*n_cols);
  handles_lines(end+1) = plot(ts,yds);
  axis tight; xlabel('t (s)'); ylabel('yd')
  title('velocity')

  subplot(n_rows,n_cols,5+2*n_cols);
  handles_lines(end+1) = plot(ts,ys);
  axis tight; xlabel('t (s)'); ylabel('y')
  title('position')

  set(handles_lines,'LineWidth',2);
  set(handles_lines,'Color',[0.4 0.4 0.8]);

  for sp=1:(n_rows*n_cols)
    if (sp==1 || sp==8 || sp==14)
    else
      subplot(n_rows,n_cols,sp);
      hold on
      ylimits = ylim;
      plot(time*ones(1,2),ylimits,'-k');
      hold off
    end
  end
  for sp=[11 12 15]
    subplot(n_rows,n_cols,sp);
    hold on
    plot(ts([1 end]),[y0 y0],'-r');
    plot(ts([1 end]),[g0 g0],'-g');
    hold off
  end


end



  function trajectory = testtransformationintegrate

    % Integrate canonical system
    dt = 1/200;
    time = 2;
    time_exec = 2*time;
    order = 3;
    [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>

    % Integrate and plot a transformation system with random weights
    n_basis_functions = 8;
    y0 = 3;
    g  = 1;
    theta = 50*randn(1,n_basis_functions);

    figure_handle = 1;

    trajectory = transformationintegrate(y0,g,theta,xs,vs,dt,time,figure_handle);

  end

end