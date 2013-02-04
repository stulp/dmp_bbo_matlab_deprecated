function update_parameters = check_update_parameters(update_parameters)
% zzz

if (nargin==0)
end

update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.eliteness           =      10;
update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
update_parameters.covar_learning_rate =       1; % No lowpass filter
update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK> % Lower relative bound 
update_parameters.covar_scales        =       1; % No scaling

end