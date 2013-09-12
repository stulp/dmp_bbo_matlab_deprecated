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

function [ task ] = task_multidofviapoint(n_dims,arm_length,arm_type)
if (nargin<1), n_dims  = 10; end
if (nargin<2), arm_length = 1; end
if (nargin<3), arm_type = 1; end

task.name = 'multidofviapoint';
task.n_dims = n_dims;
task.arm_length = arm_length;
task.link_lengths = getlinklengths(arm_type,n_dims,arm_length);

task.cost_function = @cost_function_multidofviapoint;

task.viapoint = [0.5 0.5]';
task.viapoint_time_ratio = 0.5;

% Now comes the function that does the roll-out and visualization thereof
  function costs = cost_function_multidofviapoint(task,cost_vars)

    [n_rollouts n_time_steps n_cost_vars ] = size(cost_vars); %#ok<NASGU>
    viapoint_time_step = round(task.viapoint_time_ratio*n_time_steps);
    
   for k=1:n_rollouts
      ys   = squeeze(cost_vars(k,:,1:3:end));
      ydds = squeeze(cost_vars(k,:,3:3:end));

      % Cost due to distance from viapoint
      angles = ys; % y represents angles
      % Get the end-effector position at time step 'viapoint_time_step'
      x_intermediate = getarmpos(angles,task.link_lengths,viapoint_time_step);
      % Distance of end-effector to viapoint
      dist_to_viapoint = sqrt(sum((x_intermediate-task.viapoint).^2));
      % Scale to get cost
      costs_via = dist_to_viapoint.^2;

      % Cost due to acceleration
      n_dofs = task.n_dims;
      costs_acc = zeros(size(ydds,1),1);
      sum_w = 0;
      for dof=1:n_dofs %#ok<NODEF>
        %fdd = xdd(dof); % fdd = xdd[r] - up;
        costs_acc = costs_acc + (0.5*ydds(:,dof).^2)*(n_dofs+1-dof);
        sum_w = sum_w + dof;
      end
      costs_acc = 0.0000001*costs_acc/sum_w;

      % Prepare to return the costs
      costs(k,2) = costs_via;
      costs(k,3) = sum(costs_acc); % Sum over time
      % Total cost is the sum of all the subcomponent costs
      costs(k,1) = sum(costs(k,2:end));

    end
    
  end

end