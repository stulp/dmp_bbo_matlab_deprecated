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

function covar_bounded = boundcovar(covar,bounds,scales,figure_handle)
% covar_bounded = boundcovar(covar,lower_bound,upper_bound,scales,figure_handle)
% 
% Bound a covariance matrix, i.e. enforce lower and upper bounds on its
% eigenvalues.
%
% Arguments:
%  * covar         : input covariance matrix
%  * bounds        : bounds on the eigenvalues (see below)
%  * scales        : apply a scaling to the covariance matrix before bounding it
%                      default: no scaling
%  * figure_handle : plot the result in on figure(figure_handle)
%                      default: 0, i.e. no plotting 
%
% Output:
%  * covar_bounded : covariance matrix whose eigenvalues lie within the range
%                    [lower_bound upper_bound]
%
%
% The 'bounds' argument is a 1x3 vector containing
%    1) relative lower bound - the eigenvalues may not be smaller that this
%                              factor multiplied with the largest eigenvalue
%    2) absolute lower bound - absolute lower bound on eigenvalues
%    3) absolute upper bound - absolute upper bound on eigenvalues
%    passing a shorter vector will set defaults

% Process arguments
if (nargin==0)
  % Test this function (see at the bottom of this file)
  test_boundcovar;
  return
end

if (nargin<3 || isempty(scales)), scales = 1;        end
if (nargin<4)                   , figure_handle = 0; end

% defaults
NO_BOUND = -1; % Constant
% Bounds takes form [lower_bound_relative lower_bound_absolute upper_bound_relative]
% if this vector is shorter, defaults are set
lower_bound_relative = NO_BOUND; % Default
if (~isempty(bounds)), lower_bound_relative = bounds(1); end
lower_bound_absolute = NO_BOUND; % Default
if (length(bounds)>1), lower_bound_absolute = bounds(2); end
upper_bound_absolute = NO_BOUND; % Default
if (length(bounds)>2), upper_bound_absolute = bounds(3); end  

%[ lower_bound_relative lower_bound_absolute upper_bound_absolute ]


% Scale covariance matrix
covar_scaled = covar.*(scales'*scales);

% Get eigenvalues/vectors
[eigvec,eigval] = eig(covar_scaled);
eigval = diag(eigval);

% Check for upper bound
if (upper_bound_absolute~=NO_BOUND)
  too_large = eigval>upper_bound_absolute;
  eigval(too_large) = upper_bound_absolute;
end

% Compute absolute lower bound from relative bound and maximum eigenvalue
if (lower_bound_relative~=NO_BOUND)  
  if (lower_bound_relative<0 || lower_bound_relative>1)
    warning('When using a relative lower bound, 0<=bound<=1 must hold, but it is %f. Not setting any lower bounds.',relative_lower_bound); %#ok<WNTAG>
    lower_bound_absolute = NO_BOUND;
  else
    lower_bound_absolute = max([lower_bound_absolute lower_bound_relative*max(eigval)]);
  end
end

% Check for lower bound
if (lower_bound_absolute~=NO_BOUND)
  too_small = eigval<lower_bound_absolute;
  eigval(too_small) = lower_bound_absolute;
end

% Reconstruct covariance matrix from bounded eigenvalues
eigval = diag(eigval);
covar_scaled_bounded = (eigvec*eigval)/eigvec;

% Scale covariance matrix back
covar_bounded = covar_scaled_bounded./(scales'*scales);

% Due to numerical issues, covar_bounded may contain imaginary parts. Remove.
covar_bounded = real(covar_bounded);

% Do some plotting if necessary
if (figure_handle>0 && length(covar)<4)
  figure(figure_handle)
  clf
  
  scaling_was_applied = ~all(scales==1);
  
  if (scaling_was_applied)
    % One plot in the unscaled view, one in the scaled view
    n_subplots = 2;
  else
    % No scaling: only one plot needed
    n_subplots = 1;
  end
  
  for sp=1:n_subplots
    subplot(1,n_subplots,sp)
    conf = 0.4;
    error_ellipse(covar,'conf',conf,'style','-r');
    hold on
    if (scaling_was_applied)
      error_ellipse(covar_scaled,'conf',conf,'style','--g');
      error_ellipse(covar_scaled_bounded,'conf',conf,'style','--b');
    end
    error_ellipse(covar_bounded,'conf',conf,'style','-k');
    for vv=1:length(eigval)
      if (scaling_was_applied), style='--b'; else style = '-k'; end
      plot(sqrt(eigval(vv,vv))*[0 eigvec(vv,1)],sqrt(eigval(vv,vv))*[0 eigvec(vv,2)],style)
    end
    hold off
    if (scaling_was_applied)
      legend('covar','covar-scaled','covar-scaled-bounded','covar-bounded','Location','EastOutside')
      xlabel(sprintf('x (scale = %1.2f)',scales(1)))
      ylabel(sprintf('y (scale = %1.2f)',scales(2)))
      if (sp==1)
        axis equal
        title('Unscaled space')
      else
        % Make data aspect ratio in the scaled view
        aspect_ratio = 1./scales;
        if (length(scales)==2), 
          aspect_ratio = [ aspect_ratio 1]; 
        end
        daspect(aspect_ratio)
        %pbaspect(aspect_ratio)
        title('Scaled space')
      end
    else
      axis equal
      legend('covar','covar-bounded','Location','EastOutside')
      xlabel('x')
      ylabel('y')
    end
      
  end

end

end

function test_boundcovar

% EXAMPLE 1: No scaling

covar = [40 20; 20 20];
% Make sure none of the eigenvalues drop below 10, or 0.1*max_eigval, whichever
% is larger.
% Make sure none of the eigenvalues go above 40
bounds = [0.1 10 40];
% Which figure to plot on
figure_handle = 1;

boundcovar(covar,bounds,[],figure_handle);

% EXAMPLE 2: Scaling 

% Let's say the maximum values of your two basis functions are 3 and 1
max_basis_functions = [3 1];

% The scaling is the inverse of the maximum of the basis functions.
scales = 1./abs(max_basis_functions);
scales = scales/max(scales); % Has no effect, but easier to interpret

figure_handle = 2;
boundcovar(covar,bounds,scales,figure_handle);

% EXAMPLE 3: Scaling, a more extreme example, typical of first order DMPs

bounds = [0 30]; % Lower absolute bound of 30.
max_basis_functions = [10 1];
scales = 1./abs(max_basis_functions);
scales = scales/max(scales); % Not necessary, but easier to interpret
figure_handle = 3;
boundcovar(covar,bounds,scales,figure_handle);

end

