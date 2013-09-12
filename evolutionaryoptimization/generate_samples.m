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

function samples = generate_samples(distributions,n_samples,first_is_mean)

% Input:
%   distributions - distribution from which "samples" were sampled
%                   contains fields, mean, covar, sigma (latter only for CMA-ES)
%   n_samples     - number of samples to take from distribution
%   first_is_mean - set first sample to mean of distribution (default: 1=yes)
%
% Output:
%   samples       - samples from distribution. Size:  n_dofs x n_samples x n_dims

%-------------------------------------------------------------------------------
% Call test function if called without arguments
if (nargin==0)
  samples = test_generate_samples;
  return
end
if (nargin<3)
  first_is_mean = 1;
end

n_dofs = length(distributions);
n_dims  = length(distributions(1).mean);


%-------------------------------------------------------------------------------
% Sample from new distribution
samples = zeros(n_dofs,n_samples,n_dims);
for i_dof=1:n_dofs %#ok<FXUP>
  mu = distributions(i_dof).mean;
  covar = distributions(i_dof).covar;
    
  % Sample from Gaussian for the others
  if (isequal(diag(diag(covar)),covar))
    % For diagonal covar, manual sampling is faster than using mvnrnd (older
    % versions)
    samples(i_dof,:,:) = repmat(mu,n_samples,1) + randn(n_samples,n_dims).*repmat(sqrt(diag(covar))',n_samples,1);
  else
    samples(i_dof,:,:) = mvnrnd(mu,covar,n_samples);
  end
  
  if (first_is_mean)
    % Zero exploration in first sample; useful to get performance of current mean
    samples(i_dof,1,:) = mu;
  end

end


% Main function done
%-------------------------------------------------------------------------------



%-------------------------------------------------------------------------------
% Test function
  function  samples = test_generate_samples

    % Make some distributions
    n_dofs = 3;
    n_dims = 2;
    for i_dof=1:n_dofs %#ok<FXUP>
      distributions(i_dof).mean = i_dof*ones(1,n_dims);
      distributions(i_dof).covar = i_dof*[1 0; 0 3];
    end

    % Generate some samples
    n_samples = 100;
    first_is_mean = 1;
    samples = generate_samples(distributions,n_samples,first_is_mean);
    
    % Plotting
    for i_dof=1:n_dofs %#ok<FXUP>
      subplot_handles(i_dof) = subplot(1,n_dofs,i_dof);
      plot(distributions(i_dof).mean(1),distributions(i_dof).mean(2),'ob');
      hold on
      error_ellipse(distributions(i_dof).covar,distributions(i_dof).mean);
      
      cur_samples = squeeze(samples(i_dof,:,:));
      plot(cur_samples(:,1),cur_samples(:,2),'.k');
      hold off
      axis tight
      axis equal
    end
    linkaxes(subplot_handles)
  end

end