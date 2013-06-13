function [centers widths] = basisfunctioncenters(n_basis_functions,time,alpha)

time_instead_of_phase=1;
if (nargin>2)
  % If alpha was passed, this implies you want to use phase space
  time_instead_of_phase=0;
end

if (time_instead_of_phase)
  % Basis functions centers are equidistantly spaced in time
  widths = (0.5*time/n_basis_functions)*ones(1,n_basis_functions);
  centers = linspace(0,time,n_basis_functions);
  % Time signal is time
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
  % Time signal is phase
end


end