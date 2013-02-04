function [samples_new summary distributions_new ] = update_and_sample(distributions,samples,costs,update_parameters)
% distribution: distribution from which "samples" were sampled
%               - mean
%               - covar
%               - sigma, evolution paths (only for CMA-ES)
% samples: samples from distribution. Size:  n_dofs x n_samples x n_dims
% costs: costs for each sample: 1 x n_samples
% update_parameters: parameters for updating (usually constant during optimization)
%              weighting_method   - PI-BB, CMA-ES
%              eliteness          - h (PI-BB), K_e (CMA-ES, CEM)
%              covar_update       - None, Decay, PI-BB, CMA-ES
%              covar_full         - 1: full covar update
%                                   0: diagonal only
%              covar_full         - 1: full covar update
%              covar_learing_rate - [0-1] (0 do nothing, 1 disregard previous)
%
% Special case: in the first optimization iteration, you want to sample without
% having evaluated any samples yet, i.e. costs are not yet known. For this case,
% only "distributions" are passed, and "samples" is a scalar representing the
% number of samples (rather than the samples themselves).
%
% 

%-------------------------------------------------------------------------------
% Call test function if called without arguments
if (nargin==0)
  [samples_new distributions_new ] = test_update_and_sample;
  return
end

n_dofs = length(distributions);
n_dims  = length(distributions(1).mean);

first_sampling = 0;
if (nargin<3)
  first_sampling = 1;
end

if (first_sampling)
  % Only distributions, but no samples were passed.
  % In that case, samples is a scalar representing the number of samples that
  % should be taken.
  distributions_new = distributions;
  n_samples = samples;
  
  % In this case, skip all the steps related to updating, and jump to sampling.
else

  %-------------------------------------------------------------------------------
  % Set defaults
  if (nargin<4)
    update_parameters.weighting_method    = 'PI-BB';
    update_parameters.eliteness           =      10;
    update_parameters.covar_update        = 'PI-BB';
    update_parameters.covar_full          =       0;
    update_parameters.covar_learning_rate =       1;
    update_parameters.covar_bounds        =      [];
    update_parameters.covar_scales        =       1;
  end

  if (ndims(samples)==2)
    % This function assumes samples is of size n_dofs x n_samples x n_dim
    % If is is of size n_samples x n_dim, add n_dofs=1 dimension at the front
    samples = shiftdim(samples,-1);
  end

  % How many samples do we need to take?
  n_samples = size(samples,2);
    
  %-------------------------------------------------------------------------------
  % First, map the costs to the weights
  if (strcmp(update_parameters.weighting_method,'PI-BB'))
    % PI^2 style weighting: continuous, cost exponention
    h = update_parameters.eliteness; % In PI^2, eliteness parameter is known as "h"
    weights = exp(-h*((costs-min(costs))/(max(costs)-min(costs))));

  elseif (strcmp(update_parameters.weighting_method,'CMA-ES'))
    % CMA-ES style weights: rank-based, uses defaults
    mu = update_parameters.eliteness; % In CMA-ES, eliteness parameter is known as "mu"
    [Ssorted indices] = sort(costs,'ascend');
    weights = zeros(size(costs));
    for ii=1:mu
      weights(indices(ii)) = log(mu+1/2)-log(ii);
    end

  else
    warning('Unknown weighting method number %s. Setting to "PI-BB".\n',update_parameters.weighting_method); %#ok<WNTAG>
    % Call recursively with fixed parameter
    update_parameters.weighting_method = 'PI-BB';
    [ samples_new distributions_new ] = update_and_sample(samples,costs,distribution,update_parameters,n_samples);
    return;
  end

  %-------------------------------------------------------------------------------
  % Second, compute new mean
  distributions_new = distributions;
  for i_dof=1:n_dofs
    % Update with reward-weighed averaging
    distributions_new(i_dof).mean = sum(repmat(weights,1,n_dims).*squeeze(samples(i_dof,:,:)),1);
  end

  %-------------------------------------------------------------------------------
  % Third, compute new covariance matrix
  for i_dof=1:n_dofs
    covar = distributions_new(i_dof).covar;

    if (strcmp(update_parameters.covar_update,'Decay'))
      % Decaying exploration
      covar_new = update_parameters.covar_decay*covar;

    elseif (strcmp(update_parameters.covar_update,'PI-BB'))
      % Update with reward-weighed averaging
      eps = squeeze(samples(i_dof,:,:)) - repmat(distributions(i_dof).mean,n_samples,1);
      covar_new = (repmat(weights,1,n_dims).*eps)'*eps;
      if (~update_parameters.covar_full)
        % Only use diagonal
        covar_new = diag(diag(covar_new));
      end

      % Avoid numerical issues
      covar_new = real(covar_new);

      % Apply low pass filter
      rate = update_parameters.covar_learning_rate;
      covar_new = (1-rate)*covar + rate*covar_new;

    elseif (strcmp(update_parameters.covar_update,'None'))
      % Constant exploration
      % Do nothing: covars were already copied into distributions_new above
      covar_new = covar;
    else
      if (i_dof==1) % Warn only on the first iteration.
        if (strcmp(update_parameters.covar_update,'CMA-ES'))
          warning('CMA-ES covariance matrix updating not implemented yet') %#ok<WNTAG>
        else
          warning('Unknown covariance matrix update method %s. Not updatinf covariance matrix.\n',update_parameters.covar_update); %#ok<WNTAG>
        end
      end
      covar_new = covar;
    end

    if (isempty(update_parameters.covar_bounds))
      % No bounding
      covar_new_bounded = covar_new;
    else
      covar_new_bounded = boundcovar(covar_new,update_parameters.covar_bounds,update_parameters.covar_scales);
    end

    distributions_new(i_dof).covar = covar_new_bounded;
  end

end


%-------------------------------------------------------------------------------
% Fourth, sample from new distribution
samples_new = zeros(n_dofs,n_samples,n_dims);
for i_dof=1:n_dofs
  % Zero exploration in first sample; useful to get performance of current mean
  samples_new(i_dof,1,:) = distributions_new(i_dof).mean;

  % Sample from Gaussian for the others
  samples_new(i_dof,2:end,:) = mvnrnd(distributions_new(i_dof).mean,distributions_new(i_dof).covar,n_samples-1);
end


%-------------------------------------------------------------------------------
% Bookkeeping: put relevant information in a summary
summary.distributions_new = distributions_new;
summary.samples_new = samples_new;
if (first_sampling)
  % Information below not available during first sampling
else
  summary.distributions = distributions;
  summary.samples = samples;
  summary.costs = costs;
  summary.weights = weights;
end

%-------------------------------------------------------------------------------
% Plotting
plot_me = 0;
if (plot_me)
  for i_dof=1:n_dofs
    subplot(1,n_dofs,i_dof)

    color = [0 0 1];
    plot(distributions(i_dof).mean(1),distributions(i_dof).mean(2),'o','Color',color);
    hold on
    handle = error_ellipse(distributions(i_dof).covar,distributions(i_dof).mean);
    set(handle,'Color',color);
    if (~first_sampling)
      cur_samples = squeeze(samples(i_dof,:,:));
      plot(cur_samples(:,1),cur_samples(:,2),'.','Color',color);
    end

    color = [1 0 0];
    plot(distributions_new(i_dof).mean(1),distributions_new(i_dof).mean(2),'o','Color',color);
    handle = error_ellipse(distributions_new(i_dof).covar,distributions_new(i_dof).mean);
    set(handle,'Color',color);
    cur_samples = squeeze(samples_new(i_dof,:,:));
    plot(cur_samples(:,1),cur_samples(:,2),'.','Color',color);
    hold off
    axis tight
    axis equal
  end
end

% Main function done
%-------------------------------------------------------------------------------



%-------------------------------------------------------------------------------
% Test function
  function  [samples_new summary distributions_new ] = test_update_and_sample
    n_samples = 20;
    n_dim = 2;
    center = 2;
    distributions.mean = center*ones(1,2);
    distributions.covar = eye(2);

    samples = randn(n_samples,n_dim)+center; % Gaussian with mean [center center]
    costs = sqrt(sum(samples.^2,2));

    update_parameters.weighting_method    = 'PI-BB';
    update_parameters.eliteness           =      10;
    update_parameters.covar_update        = 'PI-BB';
    update_parameters.covar_full          =       0;
    update_parameters.covar_learning_rate =       1;
    update_parameters.covar_bounds        =      [];
    update_parameters.covar_scales        =       1;

    figure(1)
    [samples_new summary distributions_new ] = update_and_sample(distributions,samples,costs,update_parameters);
    title('PI-BB, diagonal covar only')

    figure(2)
    update_parameters.covar_full          =       1;
    update_and_sample(distributions,samples,costs,update_parameters);
    title('PI-BB, full covar')

    figure(3)
    update_parameters.covar_update        = 'Decay';
    update_parameters.covar_decay         =     0.7;
    update_and_sample(distributions,samples,costs,update_parameters);
    title('Decay')

    figure(4)
    update_parameters.covar_update        = 'None';
    update_and_sample(distributions,samples,costs,update_parameters);
    title('None')

    figure(5)
    % Test initial call when no costs are available yet
    update_and_sample(distributions,n_samples);
    title('Initial sampling')

  end

end