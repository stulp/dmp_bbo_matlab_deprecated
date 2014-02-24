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

function [] = write_trajectories_to_ascii(directory,trajectories)

if (~exist(directory,'dir'))
  [status,message,messageid] = mkdir(directory); %#ok<NASGU>
end

current_update = read_current_update(directory);

output_directory = sprintf('%s/%03d_update/rollouts',directory,current_update);
if (~exist(output_directory,'dir'))
  mkdir(output_directory);
end

n_samples = length(trajectories);
for k=1:n_samples

  trajectory = trajectories(k);
    
  filename = sprintf('%s/%02d_traj_x.txt',output_directory,k);
  dlmwrite(filename,trajectory.y,' ');
  filename = sprintf('%s/%02d_traj_xd.txt',output_directory,k);
  dlmwrite(filename,trajectory.yd,' ');
  filename = sprintf('%s/%02d_traj_xdd.txt',output_directory,k);
  dlmwrite(filename,trajectory.ydd,' ');

end

filename = sprintf('%s/number_of_trials.txt',output_directory);
dlmwrite(filename,n_samples,' ');

end