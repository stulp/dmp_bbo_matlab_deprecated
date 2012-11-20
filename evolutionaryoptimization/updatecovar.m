function [ covar_new covar_new_bounded ]= updatecovar(theta,covar,theta_eps,weights,covar_update,covar_bounds,covar_lowpass,covar_scales)
%  covar_update     - determines method of covar matrix updating
%                       0     - No updating, covariance matrix does not change
%                       <0-1> - Decaying exploration
%                       1     - Update diagonal through reward-weighted averaging
%                       2     - Update full covariance matrix through reward-weighted averaging
%
%  covar_bounds     - lower/upper bounds on the covariance matrix eigenvalues
%                       Takes form [lower_relative lower_absolute upper_absolute]
%                       lower_relative - none of the covariance matrix'
%                                        eigenvalues may be smaller than
%                                        lower_relative times the largest
%                                        eigenvalue
%                                        default: 0.01
%                       lower_absolute - none of the covariance matrix'
%                                        eigenvalues may be smaller than
%                                        lower_absolute
%                                        default: no bound
%                       upper_absolute - none of the covariance matrix'
%                                        eigenvalues may be smaller than
%                                        upper_absolute
%                                        default: no bound
%
%
%
%  covar_lowpass  - covar_new = (1-covar_lowpass)*covar + covar_lowpass*covar_new;
%
%  covar_scales - scaling factor for covar

if (nargin<5), covar_update  = 0.9; end
if (nargin<6), covar_bounds  =  []; end
if (nargin<7), covar_lowpass =   0; end
if (nargin<8), covar_scales  =   1; end
  
  
if (covar_update>0 && covar_update<1)
  % Decaying exploration
  covar_new = covar_update*covar;

elseif (covar_update>=1)
  % Update with reward-weighed averaging
  [ K n_dim ] = size(theta_eps);
  eps = theta_eps - repmat(theta,K,1);
  covar_new = (repmat(weights,1,n_dim).*eps)'*eps;
  if (covar_update==1)
    % Only use diagonal
    covar_new = diag(diag(covar_new));
  end

  % Avoid numerical issues
  covar_new = real(covar_new);
  
  % Apply low pass filter
  covar_new = (1-covar_lowpass)*covar_new + covar_lowpass*covar;


else
  % Constant exploration
  covar_new = covar;
end

if (isempty(covar_bounds))
  % No bounding
  covar_new_bounded = covar_new;
else
  covar_new_bounded = boundcovar(covar_new,covar_bounds,covar_scales);
end

covar_new = covar_new_bounded;
end