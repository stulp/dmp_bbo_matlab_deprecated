function [ theta y0 g ] = dmptrain(trajectory,order,n_basis_functions,figure_handle)
% Train a Dynamic Movement Primitive with an observed trajectory
%
% Input:
%   trajectory    - the trajectory with which to train 
%                   this is a structure that contains
%                      trajectory.t   - times (T x 1)
%                      trajectory.y   - position over time (T x n_trans)
%                      trajectory.yd  - velocity over time (T x n_trans)
%                      trajectory.ydd - acceleration over time (T x n_trans)
%   order             - order of the canonical system
%   n_basis_functions - number of basis functions (same for each transfsys)
%   figure_handle     - whether to plot, default = 0
%   
% Output:
%   theta  - DMP weight parameters (n_trans x n_basis_functions)
%   y0     - DMP initial state (1 x n_trans)
%   g      - DMP goal state (1 x n_trans)

if (nargin==0)
  % If no arguments are passed, test the function
  [ theta y0 g ] = testdmptrain;
  return;
end

%-------------------------------------------------------------------------------
% Default values
if (nargin<4), figure_handle = 0; end

%-------------------------------------------------------------------------------
% Integrate canonical system in closed form
time = trajectory.t(end);
time_exec = time;
dt = mean(diff(trajectory.t));
[ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>

% Prepare output variables
n_trans = size(trajectory.y,2);
theta = zeros(n_trans,n_basis_functions);
y0 = zeros(1,n_trans);
g = zeros(1,n_trans);

%-------------------------------------------------------------------------------
% Each dimension integrated separately
for i_trans = 1:n_trans

  % Train this transformation system
  if (figure_handle)
    figure_handle_per_dim = figure_handle+i_trans-1;
  else
    figure_handle_per_dim = 0;
  end
  
  trajectory_per_dim.t   = trajectory.t; 
  trajectory_per_dim.y   = trajectory.y(:,i_trans); 
  trajectory_per_dim.yd  = trajectory.yd(:,i_trans); 
  trajectory_per_dim.ydd = trajectory.ydd(:,i_trans); 
  
  [ theta(i_trans,:) y0(i_trans) g(i_trans) ] =  transformationtrain(trajectory_per_dim,n_basis_functions,xs,vs,time,figure_handle_per_dim);
 
end

if (figure_handle)
  figure(figure_handle+n_trans)

  % Reproduce the trajectory
  time_exec = 1.25*time;
  [ trajectory ] = dmpintegrate(y0,g,theta,time,dt,time_exec,order);
  hold on
  if (n_trans>2)
    handle = plot3(trajectory.y(:,1,1),trajectory.y(:,2,1),trajectory.y(:,3,1));
  else
    handle = plot(trajectory.y(:,1,1),trajectory.y(:,2,1));
  end  
  hold off

  % Make reproduced trajectories thin and dark
  set(handle,'LineWidth',1);
  set(handle,'Color',[0.0 0.0 0.5]);
end  
    


  function [ theta y0 g0 ] = testdmptrain

    % Get a trajectory by integrating a DMP
    n_basis_functions = 8;
    n_trans = 2;
    theta = 25*randn(n_trans,n_basis_functions);

    y0 = [ 1.3 0.1];
    g  = [-2.0 0.9];
    time = 2;
    dt = 1/100;
    time_exec = time;
    order = 3;
    
    % Generate a trajectory
    figure_handle = 1;
    [ trajectory activations canonical_at_centers handle ] = dmpintegrate(y0,g,theta,time,dt,time_exec,order,figure_handle);
    % Make original trajectories thick and bright
    set(handle,'LineWidth',3);
    set(handle,'Color',[0.7 0.7 1]);
    
    % Train a DMP with this trajectory
    [ theta y0 g0 ] = dmptrain(trajectory,order,n_basis_functions,figure_handle);

  end


end