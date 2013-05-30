function [ts xs xds vs vds alpha] = canonicalintegrate(time,dt,time_exec,order,alpha)
% Integrate a canonical system for 'ceil(1+time_exec/dt)' time steps of duration 'dt'
%
% This function uses the closed-form solution of the dynamical system 
% representing the canonical system, so it is fast.
%
% Input:
%   time          - duration of the observed movement
%   dt            - duration of the integration step
%   time_exec     - duration of the integration
%   order         - order of the canonical system (1 or 2)
%   alpha         - time constant, determined speed of convergence
% Output:
%   ts            - time over time ;-)
%   xs,xds,vs,vds - state of the canonical system over time
%   alpha         - time constant, determines speed of convergence

if (nargin==0)
  % If no arguments are passed, test the function
  [ts xs xds vs vds] = testcanonicalintegrate;
  return;
end

% Set some defaults
if (nargin<3), time_exec = time; end
if (nargin<4), order = 2; end
if (nargin<5), 
  if (order==1)
    alpha = -log(0.001);
  else
    alpha = 15;
  end
end

N = ceil(1+time_exec/dt); % Number of time steps
ts = dt*(0:N-1)'; % time over time
if (order==1)
  % Closed form solution to 1st order canonical system 
  xs  = exp(-alpha*ts/time);
  xds = -(alpha/time)*exp(-alpha*ts/time);
  vs = xs;
  vds = xds;

elseif (order==2)
  % Closed form solution to 2nd order canonical system 
  % This system behaves like a critically damped spring-damper system
  % See http://en.wikipedia.org/wiki/Damped_spring-mass_system#Example:_mass.E2.80.93spring.E2.80.93damper
  
  % Must hold for system to be critically damped
  beta = alpha/4;

  m=1;           % mass
  c=alpha;       % damping coefficient
  k=alpha*beta;  % spring constant

  omega_0 = sqrt(k/m);     % natural frequency
  zeta = c/(2*sqrt(m*k));  % damping ratio
  if (zeta~=1)
    warning('Canonical system is not critically damped (zeta=%f)',zeta) %#ok<WNTAG>
  end

  % initial conditions
  x0 = 1;
  xd0 = 0;

  % these values represent the solution to the differential equation
  A = x0;
  B = xd0 + omega_0*x0;

  % Closed form solution
  xs = (A+(B/time)*ts).*exp((-omega_0/time)*ts);
  xds = (B/time).*exp((-omega_0/time)*ts) + (-omega_0/time)*(A+(B/time)*ts).*exp((-omega_0/time)*ts);
  %vs = xds*time;
  vs = B.*exp((-omega_0/time)*ts) -omega_0*(A+(B/time)*ts).*exp((-omega_0/time)*ts);
  vds = (-omega_0/time)*B.*exp((-omega_0/time)*ts) - omega_0*(B/time).*exp((-omega_0/time)*ts) -omega_0*(-omega_0/time)*(A+(B/time)*ts).*exp((-omega_0/time)*ts);
end



  function [ts xs xds vs vds] = testcanonicalintegrate

    alpha = 14;
    time = 2;
    time_exec = 2; % 2.5;
    dt = 1/100;
    labels = {'vd','v','xd','x'};

    for canonical_order=1:2
      [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,canonical_order,alpha);

      % Plot
      figure(canonical_order)
      clf
      subplot(2,2,1); plot(ts,vds,'--','LineWidth',2,'Color',[0 0 0]);
      title(['System of order ' num2str(canonical_order)])
      subplot(2,2,2); plot(ts,vs, '--','LineWidth',2,'Color',[0 0 0]);
      subplot(2,2,3); plot(ts,xds,'--','LineWidth',2,'Color',[0 0 0]);
      subplot(2,2,4); plot(ts,xs, '--','LineWidth',2,'Color',[0 0 0]);
      for dd=1:4
        subplot(2,2,dd);
        xlabel('t (s)')
        ylabel(labels(dd))
      end
    end
  end
end



