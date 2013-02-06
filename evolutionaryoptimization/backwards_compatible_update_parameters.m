function update_parameters = backwards_compatible_update_parameters(eliteness,further_args)
% Convert previous passing of arguments to evolutionaryoptimization to new
% update_parameters structure.

% Get defaults 
update_parameters = check_update_parameters;

% Override defaults with arguments (if available)
update_parameters.eliteness           =      eliteness;

if (length(further_args)>=1)
  %  1 - Cross-entropy method weights
  %  2 - CMAES default weights
  %  3 - PI^2 weights
  weighting_method_names = {'CEM','CMA-ES','PI-BB'};
  weighting_method_index = further_args{1};
  update_parameters.weighting_method    = weighting_method_names{weighting_method_index};
end

if (length(further_args)>=2)
  covar_update = further_args{2};
  if (covar_update<1)
    update_parameters.covar_update =      'decay';
    update_parameters.covar_decay  = covar_update;
  else
    update_parameters.covar_update = 'PI-BB';
    update_parameters.covar_full   =       (covar_update==2);
  end
end

if (length(further_args)>=3)
  update_parameters.covar_bounds        = further_args{3};
end
if (length(further_args)>=4)
  update_parameters.covar_learning_rate = further_args{4};
end
if (length(further_args)>=5)
  update_parameters.covar_scales        = further_args{5};
end


end