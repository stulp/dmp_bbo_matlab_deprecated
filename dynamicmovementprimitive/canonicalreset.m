function canon_system = canonicalreset(order,alpha)
% Reset a canonical system
%
% Input:
%   order - order of the canonical system (1 or 2)
%   alpha - time constant, determined speed of convergence
% Output:
%   canon_system - initialized canonical system, containing
%                    * order, alpha, beta
%                    * v, vd, x, xd
% 

% Set some defaults
if (nargin<1) order =    2; end
if (nargin<2) alpha = 10.0; end

canon_system.order = order;
canon_system.alpha = alpha;
canon_system.beta  = alpha/4;

% Initialize canonical system
if (canon_system.order==1)
  canon_system.v=1;
else
  canon_system.v=0;
end
canon_system.vd=0;
canon_system.x=1;
canon_system.xd=0;

end
