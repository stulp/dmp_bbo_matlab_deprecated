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

function [] = write_dmp_parameters_to_ascii(directory,thetas)

if (~exist(directory,'dir'))
  mkdir(directory);
end

current_update = read_current_update(directory);

if (ndims(thetas)==2)
  thetas = shiftdim(thetas,-1);
end
n_samples = size(thetas,2);

output_directory = sprintf('%s/%03d_update/rollouts',directory,current_update);
if (~exist(output_directory,'dir'))
  mkdir(output_directory);
end

filename = sprintf('%s/number_of_trials.txt',output_directory);
dlmwrite(filename,n_samples,' ');

for k=1:n_samples
  theta = squeeze(thetas(:,k,:));

  filename = sprintf('%s/%02d_dmpparameters.txt',output_directory,k);
  dlmwrite(filename,theta,' ');

end
end