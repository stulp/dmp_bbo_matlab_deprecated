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

function activations = basisfunctionactivations(centers,widths,xs)
% Compute basis activations for 1 or more time steps
%
% Input:
%   centers - centers of the basis functions
%   widths  - widths of the basis functions
%   xs      - if scalar: current phase (or time)
%             if vector: sequence of phases (or time)
% Output:
%  activations - activations of the basis functions at each time step

if (nargin==0)
  % If no arguments are passed, test the function
  activations  = testbasisfunctionactivations;
  return;
end

n_basis_functions = length(centers);
activations = zeros(length(xs),n_basis_functions);
for bb=1:n_basis_functions
  activations(:,bb) = exp((-0.5/(widths(bb).^2)) * (xs - centers(bb)).^2);
end


%-------------------------------------------------------------------------------
  function activations  = testbasisfunctionactivations
    clf

    time = 2;
    time_exec = 2.5;
    dt = 1/50;

    % Get time and phase vector
    order = 1;
    [ts xs xds vs vds alpha] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>
    N = ceil(1+time_exec/dt); % Number of time steps
    ts = dt*(0:N-1)';

    n_basis_functions = 10;
    for time_instead_of_phase=0:1
      if (time_instead_of_phase)
        ps = ts;
        [centers widths] = basisfunctioncenters(n_basis_functions,time);
      else
        ps = xs;
        [centers widths] = basisfunctioncenters(n_basis_functions,time,alpha);
      end

      % Get activations
      activations = basisfunctionactivations(centers,widths,ps);
      subplot(2,5,1+time_instead_of_phase*5)
      plot(ts,activations')
      xlabel('t (s)')
      ylabel('activations')
      axis tight
      if (time_instead_of_phase)
        title('In time space')
      else
        title('In phase space')
      end
      ylim([0 1.1])

      for order = 0:3

        if (order==0)
          % 'Fake' the canonical system (not used for order 0)
          vs = ones(size(ts));
        else
          [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>
        end

        % Plot basis functions, multiplied with canonical system
        subplot(2,5,2+order+time_instead_of_phase*5)
        plot(ts,repmat(vs,1,n_basis_functions).*activations)
        hold on
        plot(ts,vs,'-k','LineWidth',2)
        hold off
        xlabel('t (s)')
        ylabel('activation * v')
        axis tight
        labels = {'No canonical','1st order','2nd order','sigmoid'};
        title(labels{order+1})
        if (order~=2)
          ylim([0 1.1])
        end

      end
    end
  end


end
