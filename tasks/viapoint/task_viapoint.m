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

function [task] = task_viapoint(viapoint,viapoint_time_ratio)
if (nargin<1), viapoint  = [0.4 0.7]; end
if (nargin<2), viapoint_time_ratio = 0.5; end

task.name = 'viapoint';

task.viapoint = viapoint;
task.viapoint_time_ratio = viapoint_time_ratio;

task.cost_function= @cost_function_viapoint;


  function costs = cost_function_viapoint(task,cost_vars)
    
    [n_rollouts n_time_steps n_cost_vars ] = size(cost_vars); %#ok<NASGU>
    viapoint_time_step = round(task.viapoint_time_ratio*n_time_steps);
    
    ys = zeros(n_time_steps,(n_cost_vars-1)/3);
    ydds = zeros(n_time_steps,(n_cost_vars-1)/3);
    for k=1:n_rollouts
      ys(:,:)   = squeeze(cost_vars(k,:,1:3:end-1));
      ydds(:,:) = squeeze(cost_vars(k,:,3:3:end-1));
      
      if (task.viapoint_time_ratio<0)
        % Choose the closes point along the trajectory
        dist_to_viapoint = min(sqrt(sum((ys - repmat(task.viapoint,n_time_steps,1)).^2,2)));
      else
        dist_to_viapoint = sqrt(sum((ys(viapoint_time_step,:)-task.viapoint).^2));
      end
      costs(k,2) = dist_to_viapoint;

      % Cost due to acceleration
      sum_ydd = sum((sum(ydds.^2,2)));
      costs(k,3) = 0*(sum_ydd/10000);

      % Cost due to norm of parameters (regularization)
      norm_theta = squeeze(cost_vars(k,1,end));
      costs(k,4) = norm_theta/100;

      % Total cost is the sum of all the subcomponent costs
      costs(k,1) = sum(costs(k,2:end));
      
    end
  end

end

