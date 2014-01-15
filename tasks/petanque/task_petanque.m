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

function [task] = task_petanque(goal_ball)
if (nargin<1), goal_ball = zeros(1,4); end

task.name = 'petanque';
task.goal_ball = goal_ball;
task.cost_function = @cost_function_petanque;

addpath dynamicmovementprimitive/

% Now comes the function that does the roll-out and visualization thereof
  function [ costs cost_vars ] = cost_function_petanque(task,all_cost_vars)
   
    n_samples = size(all_cost_vars,1);
    for k=1:n_samples
      cost_vars = squeeze(all_cost_vars(k,:,:));
      
      ball_goal=  cost_vars(end,1:3);
      ball_landed =  cost_vars(end,4:6);
      
      dist = sqrt(sum((ball_landed-ball_goal).^2));
      cost = dist*dist;

      costs(k,:) = cost;
    end
  end
    

end

