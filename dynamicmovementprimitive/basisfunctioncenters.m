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

function [centers widths] = basisfunctioncenters(n_basis_functions,time,alpha)
% Get the equidistant centers and widths of a set of basis functions
%
%
% Input:
%   n_basis_functions - number of basis functions
%   time              - length of the movement in time
%   alpha             - canonical system parameter
%                       optional: if it is passed, the basis functions are
%                       placed in phase space such that they are equidistantly
%                       placed in time space.
%
% Output:
%   centers - centers of the basis functions
%   widths  - widths of the basis functions

time_instead_of_phase=1;
if (nargin>2)
  % If alpha was passed, this implies you want to use phase space
  time_instead_of_phase=0;
end

if (time_instead_of_phase)
  % Basis functions centers are equidistantly spaced in time
  widths = (0.5*time/n_basis_functions)*ones(1,n_basis_functions);
  centers = linspace(0,time,n_basis_functions);
else
  % Basis functions centers are approximately equidistantly spaced in phase space 1->0
  % Centers in time space (1 extra to be able to do diff later)
  centers_time = time/(n_basis_functions-1)*(0:n_basis_functions);
  % Compute centers in phases space from equidistant centers in time space
  centers = exp(-alpha*centers_time/time);
  % Compute widths from centers
  widths = 0.5*abs(diff(centers));
  % Remove the extra center used to compute the widths with diff
  centers  = centers(1:end-1);
end


end