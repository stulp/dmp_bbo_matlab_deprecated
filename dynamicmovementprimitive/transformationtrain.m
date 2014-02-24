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

function [ theta y0 g0 ] = transformationtrain(trajectory,n_basis_functions,xs,vs,time,figure_handle)
% Train one transformation system of a Dynamic Movement Primitive
%
% Input:
%   trajectory    - the trajectory with which to train
%                   this is a structure that contains
%                      trajectory.t   - times (T x 1)
%                      trajectory.y   - position over time (T x 1)
%                      trajectory.yd  - velocity over time (T x 1)
%                      trajectory.ydd - acceleration over time (T x 1)
%   n_basis_functions - number of basis functions
%   xs,vs             - output of the canonical system
%   time              - duration of the observed movement
%   figure_handle     - whether to plot, default = 0
%
% Output:
%   theta  - DMP weight parameters (1 x n_basis_functions)
%   y0     - DMP initial state (1x1)
%   g      - DMP goal state (1x1)

if (nargin==0)
  % If no arguments are passed, test the function
  [ theta y0 g0 ] = testtransformationtrain;
  return;
end

%-------------------------------------------------------------------------------
% Default values
if (nargin<6), figure_handle     = 0; end



%-------------------------------------------------------------------------------
% Initialization
y0 = trajectory.y(1);
g0  = trajectory.y(end);

% Compute target non-linear component f
alpha_z = 14.0;
beta_z = alpha_z/4.0;
alpha_g = alpha_z/2.0;

gs  = g0+exp(-alpha_g*trajectory.t/time)*(y0-g0);

f_target = (-alpha_z*(beta_z*(gs-trajectory.y)-trajectory.yd) + time*trajectory.ydd)/(g0-y0);

%-------------------------------------------------------------------------------
% Compute basis function activations
time_instead_of_phase=1;
if (time_instead_of_phase)
  % Time signal is time
  ps = trajectory.t;
  % Get centers and widths
  [centers widths] = basisfunctioncenters(n_basis_functions,time);
else
  % Time signal is phase
  ps = xs;
  % Reconstruct alpha
  dt=mean(diff(trajectory.t));
  alpha = -time*log(xs(2))/dt;
  % Get centers and widths
  [centers widths] = basisfunctioncenters(n_basis_functions,time,alpha);
end

% Compute activations
activations = basisfunctionactivations(centers,widths,ps);

%-------------------------------------------------------------------------------
% Compute the regression, using linear least squares
%   (http://en.wikipedia.org/wiki/Linear_least_squares)
vs_repmat = repmat(vs',n_basis_functions,1);
sum_activations = repmat(sum(abs(activations),2)',n_basis_functions,1);
activations_normalized = activations' ./ sum_activations;
vs_activ_norm = vs_repmat.*activations_normalized;
small_diag = diag(ones(n_basis_functions,1)*1e-10);
AA = inv(vs_activ_norm*vs_activ_norm' + small_diag)*vs_activ_norm ;
theta = (AA * f_target)';


% Plot if necessary
if (figure_handle)
  figure(figure_handle)
  clf

  % Execute and plot with the theta we have just computed
  dt = mean(diff(trajectory.t));
  [traj handles ] = transformationintegrate(y0,g0,theta,xs,vs,dt,time,figure_handle);

  n_rows = 3;
  n_cols = 5;
  h = [];

  ts = trajectory.t;

  % Overlay the train trajectory
  subplot(n_rows,n_cols,5+0*n_cols);
  hold on
  h(end+1) = plot(ts,trajectory.ydd);
  hold off

  subplot(n_rows,n_cols,5+1*n_cols);
  hold on
  h(end+1) = plot(ts,trajectory.yd);
  hold off

  subplot(n_rows,n_cols,5+2*n_cols);
  hold on
  h(end+1) = plot(ts,trajectory.y);
  hold off

  % Make original trajectories thick and bright
  set(handles,'LineWidth',3);
  set(handles,'Color',[0.7 0.7 1]);

  % Make reproduced trajectories thin and dark
  set(h,'LineWidth',1);
  set(h,'Color',[0.0 0.0 0.5]);

  set(gcf,'Name','Transformation system training');

end



  function [ theta y0 g ] = testtransformationtrain

    % Integrate canonical system
    dt = 1/250;
    time = 2;
    time_exec = 3;
    order = 3;
    [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>

    % Integrate and plot a transformation system with random weights
    y0 = 5;
    g  = 4;
    n_basis_functions=8;
    theta_known = 50*randn(1,n_basis_functions);

    trajectory = transformationintegrate(y0,g,theta_known,xs,vs,dt,time);

    figure_handle = 1;
    [ theta y0 g0 ] = transformationtrain(trajectory,n_basis_functions,xs,vs,time,figure_handle);

    n_rows=3;
    n_cols=5;
    subplot(n_rows,n_cols,3+1*n_cols);
    hold on
    h = stem(theta_known);
    set(h,'LineWidth',1);
    set(h,'Color',[0.0 0.0 0.5]);
    hold off

  end

end