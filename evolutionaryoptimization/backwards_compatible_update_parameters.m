function update_parameters = backwards_compatible_update_parameters(eliteness,varargin)

update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
update_parameters.covar_learning_rate =       1; % No lowpass filter
update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK> % Lower relative bound 
update_parameters.covar_scales        =       1; % No scaling

update_parameters.eliteness           =      eliteness;
if (length(varargin)>=1)
  update_parameters.weighting_method    = varargin{1};
end
if (length(varargin)>=2)
  update_parameters.covar_update        = varargin{2};
end
if (length(varargin)>=3)
  update_parameters.covar_bounds        = varargin{3};
end
if (length(varargin)>=4)
  update_parameters.covar_learning_rate = varargin{4};
end
if (length(varargin)>=5)
  update_parameters.covar_scales        = varargin{5};
end

update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full

end