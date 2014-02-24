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

function [ts xs xds vs vds alpha] = canonicalintegrate(time,dt,time_exec,order,alpha)
% Integrate a canonical system for 'ceil(1+time_exec/dt)' time steps of duration 'dt'
%
% This function uses the closed-form solution of the dynamical system
% representing the canonical system.
%
% Input:
%   time          - duration of the observed movement
%   dt            - duration of the integration step
%   time_exec     - duration of the integration
%   order         - order of the canonical system (1 or 3)
%   alpha         - time constant, determined speed of convergence
% Output:
%   ts            - time over time, i.e. 0.01, 0.02, 0.03 etc
%   xs,xds,vs,vds - state of the canonical system over time
%   alpha         - time constant, determines speed of convergence

if (nargin==0)
  % If no arguments are passed, test the function
  [ts xs xds vs vds alpha] = testcanonicalintegrate;
  return;
end

% Set some defaults
if (nargin<3), time_exec = time; end
if (nargin<4), order = 2; end
if (nargin<5),
  if (order==1)
    % Set alpha such that the canonical system is 0.001 at the end of the
    % movement
    alpha = -log(0.001);
  else
    % Set alpha to 15, as we always do.
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
elseif (order==3)

  % Closed form solution to 1st order canonical system
  alpha = -log(0.001);
  xs  = exp(-alpha*ts/time);
  xds = -(alpha/time)*exp(-alpha*ts/time);

  % Sigmoid for the hull
  final = 0.01;
  sigmoid_time = 0.25*time;
  c = log((1/final)-1)/(0.5*sigmoid_time);
  vs = 1-(1./(1+exp(-c*(ts-time+0.5*sigmoid_time))));%.^(1/4);
  vds = -c*(1./(1+exp(-c*(ts-time+0.5*sigmoid_time)))).*(1-1./(1+exp(-c*(ts-time+0.5*sigmoid_time))));

end


%-------------------------------------------------------------------------------
  function [ts xs xds vs vds alpha] = testcanonicalintegrate

    alpha = 14;
    time = 2;
    time_exec = 2; % 2.5;
    dt = 1/100;
    labels = {'vd','v','xd','x'};

    for canonical_order=1:3
      [ts xs xds vs vds] = canonicalintegrate(time,dt,time_exec,canonical_order,alpha);

      % Plot
      figure(canonical_order)
      clf
      set(gcf,'Name',['System of order ' num2str(canonical_order)]);
      subplot(2,2,1); plot(ts,xds,'--','LineWidth',2,'Color',[0 0 0]);
      subplot(2,2,2); plot(ts,vds,'--','LineWidth',2,'Color',[0 0 0]);
      subplot(2,2,3); plot(ts,xs, '--','LineWidth',2,'Color',[0 0 0]);
      subplot(2,2,4); plot(ts,vs, '--','LineWidth',2,'Color',[0 0 0]);
      for dd=1:4
        subplot(2,2,dd);
        xlabel('t (s)')
        ylabel(labels(dd))
      end
    end
  end
end



