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

function update_parameters = check_update_parameters(update_parameters)
if (nargin==0)
  update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
  update_parameters.eliteness           =      10;
  update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
  update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
  update_parameters.covar_learning_rate =       1; % No lowpass filter
  update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK> % Lower relative bound
  update_parameters.covar_scales        =       1; % No scaling
  update_parameters.first_is_mean       =       1; % The first sample is the mean
end

% Do some checks here
if (strcmp(update_parameters.covar_update,'decay'));
  default_decay = 0.95;
  if (~isfield(update_parameters,'covar_decay'))
    warning('covar update method is decay, but no decay factor set. Setting to default: %1.2f',default_decay) %#ok<WNTAG>
    update_parameters.covar_decay = default_decay;
  else
    if (update_parameters.covar_decay<=0 || update_parameters.covar_decay>=1)
      warning('covar decay must be in range <0-1>, but it is %1.2f. Setting to default: %1.2f',update_parameters.covar_decay,default_decay) %#ok<WNTAG>
    end
  end

end

if (~isfield(update_parameters,'covar_scales'))
  update_parameters.covar_scales        =       1; % No scaling
end

if (~isfield(update_parameters,'first_is_mean'))
  update_parameters.first_is_mean        =       1; % The first sample is the mean
end


end