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

function cost_vars = read_costvars_from_ascii(directory,current_update)
    
if (nargin<2)
  filename = sprintf('%s/current_update.txt',directory);
  current_update = load(filename);
end

output_directory = sprintf('%s/%03d_update/rollouts',directory,current_update);

filename = sprintf('%s/number_of_trials.txt',output_directory);
n_samples = load(filename);

for i_sample=1:n_samples
  filename = sprintf('%s/%02d_costvars.txt',output_directory,i_sample);
  cost_vars(i_sample,:,:) = load(filename);
end

% Increment update counter
filename = sprintf('%s/current_update.txt',directory);
dlmwrite(filename,current_update+1,' ');

end