function update_parameters = check_update_parameters(update_parameters)
if (nargin==0)
  update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
  update_parameters.eliteness           =      10;
  update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
  update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
  update_parameters.covar_learning_rate =       1; % No lowpass filter
  update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK> % Lower relative bound
  update_parameters.covar_scales        =       1; % No scaling
end

% Do some checks here
if (strcmp(update_parameters.covar_update,'decay'));
  default_decay = 0.95;
  if (~isfield(update_parameters,'decay'))
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


end