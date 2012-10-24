function activations = basisfunctionactivations(centers,widths,xs)
% Compute basis activations for 1 or more time steps
%
% Input:
%   centers - centers of the basis functions in phase space
%   widths  - widths of the basis functions in phase space
%   xs      - if scalar: current phase (or time)
%             if vector: sequence of phases (or time)
% Output: 
%  activations - activations of the basis functions at each phase/time step

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



  function activations  = testbasisfunctionactivations
    clf
    
    time = 2;
    time_exec = 2.5;
    dt = 1/25;
    n_basis_functions = 10;
    
    for order = 0:2

      if (order==0)
        
        % Order 0 means a policy without a dynamical system
        % Basis functions centers are equidistantly spaced in time
        widths = (0.4*time/n_basis_functions)*ones(1,n_basis_functions);
        centers = linspace(4*widths(1),time-4*widths(1),n_basis_functions);
        %centers = linspace(0,time,n_basis_functions);

        % Time vector
        N = ceil(1+time_exec/dt); % Number of time steps
        ts = dt*(0:N-1)';
        
        % Get activations
        activations = basisfunctionactivations(centers,widths,ts);
        
        % 'Fake' the canonical system (not used for order 0)
        vs = ones(size(ts));
        
      else
        
        % Order 1/2 means a policy WITH a dynamical system
        % Basis functions centers are equidistantly spaced in phase space 1->0
        centers = linspace(1,0.001,n_basis_functions);
        widths = ones(size(centers))/n_basis_functions;
        
        % Time and phase vector
        [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,order); %#ok<NASGU>
        
        % Get activations
        activations = basisfunctionactivations(centers,widths,xs);
        
      end

      % Plot basis functions
      subplot(2,3,order+1)
      plot(ts,activations')
      xlabel('t (s)')
      ylabel('activation')
      axis tight
      title(['System of order ' num2str(order)])

      % Plot basis functions, multiplied with canonical system
      subplot(2,3,order+4)
      plot(ts,repmat(vs,1,n_basis_functions).*activations)
      hold on
      plot(ts,vs,'-k','LineWidth',2)
      hold off
      xlabel('t (s)')
      ylabel('activation * v')
      axis tight

    end
  end

end
