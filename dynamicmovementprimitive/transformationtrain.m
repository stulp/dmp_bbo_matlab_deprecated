function [ theta y0 g ] = transformationtrain(trajectory,n_basis_functions,xs,vs,figure_handle)
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
%   figure_handle     - whether to plot, default = 0
%   
% Output:
%   theta  - DMP weight parameters (1 x n_basis_functions)
%   y0     - DMP initial state (1x1)
%   g      - DMP goal state (1x1)

if (nargin==0)
  % If no arguments are passed, test the function
  [ theta y0 g ] = testtransformationtrain;
  return;
end

%-------------------------------------------------------------------------------
% Default values
%if (nargin<2), n_basis_functions = 8; end
%if (nargin<3), canonical_order   = 2; end
%if (nargin<4), canonical_alpha  = 15; end
if (nargin<5), figure_handle     = 0; end



%-------------------------------------------------------------------------------
% Initialization
y0 = trajectory.y(1);
g  = trajectory.y(end);

% Compute target non-linear component f
alpha_z = 14.0;
beta_z = alpha_z/4.0;
f_target = (-alpha_z*(beta_z*(g-trajectory.y)-trajectory.yd) + trajectory.ydd)/(g-y0);

%-------------------------------------------------------------------------------
% Get basis function activations
centers = linspace(1,0.001,n_basis_functions);
widths = ones(size(centers))/n_basis_functions;
activations = basisfunctionactivations(centers,widths,xs);

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
  [traj handles ] = transformationintegrate(y0,g,theta,xs,vs,dt,figure_handle);

  n_rows = 3;
  n_cols = 4;
  h = [];
  
  ts = trajectory.t;
  
  % Overlay the train trajectory 
  subplot(n_rows,n_cols,4+0*n_cols);
  hold on
  h(end+1) = plot(ts,trajectory.ydd);
  hold off

  subplot(n_rows,n_cols,4+1*n_cols);
  hold on
  h(end+1) = plot(ts,trajectory.yd);
  hold off

  subplot(n_rows,n_cols,4+2*n_cols);
  hold on
  h(end+1) = plot(ts,trajectory.y);
  hold off
  
  % Make original trajectories thick and bright
  set(handles,'LineWidth',3);
  set(handles,'Color',[0.7 0.7 1]);

  % Make reproduced trajectories thin and dark
  set(h,'LineWidth',1);
  set(h,'Color',[0.0 0.0 0.5]);

end



  function [ theta y0 g ] = testtransformationtrain

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
    theta_known = 100*randn(1,n_basis_functions);
    
    trajectory = transformationintegrate(y0,g,theta_known,xs,vs,dt);
    
    figure_handle = 1;
    [ theta y0 g ] = transformationtrain(trajectory,n_basis_functions,xs,vs,figure_handle);
    
    n_rows=3;
    n_cols=4;
    subplot(n_rows,n_cols,2+1*n_cols);
    hold on
    h = stem(theta);
    set(h,'LineWidth',1);
    set(h,'Color',[0.0 0.0 0.5]);
    hold off

  end

end