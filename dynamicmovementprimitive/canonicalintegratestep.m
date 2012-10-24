function canon_system = canonicalintegratestep(time,dt,canon_system)
% Integrate a canonical system for 1 time step of duration 'dt'
%
% Input:
%   time         - duration of the movement
%   dt           - duration of the integration step
%   canon_system - current state of the canonical system
% Output:
%   canon_system - next state of the canonical system (after 1 integration step)

if (nargin==0)
  % If no arguments are passed, test the function
  canon_system = testintegratecanonicalstep;
  return;
end

% Just to avoid long variable names in this function
c = canon_system;

% Integrate vd and xd to get v and x
% (done before compute vd and xd to avoid non-zero v_0 for 2nd order system)
c.v  = c.v + c.vd*dt;
c.x  = c.x + c.xd*dt;

% Integrate to get vd and xd
if (c.order==1)
  c.xd = -c.alpha*c.x/time;
  c.vd = c.xd;
  %ts = linspace(0,time,1+time/tau);
  %xs = exp(-alpha_x*ts)';
  %xds = -alpha_x*exp(-alpha_x*ts)';

elseif (c.order==2)
  c.vd = canon_system.alpha*(-canon_system.beta*canon_system.x - canon_system.v)/time;
  c.xd = c.v/time;

else
  error('Dont know canonical system of order=%d',c.order)

end

% Just to avoid long variable names in this function
canon_system = c;


  function canon_system  = testintegratecanonicalstep
    time = 2;
    time_exec = 2.5;
    dt = 1/100;
    labels = {'vd','v','xd','x'};

    N = 1+time_exec/dt; % Number of time steps
    t = dt*(0:N-1);
    for order=1:2 % Plot for both 1st and 2nd order 
      % Initialize
      alpha = 6;
      canon_system = canonicalreset(order,alpha);

      % Prepare matrix with relevant information, i.e. xs, xds, vs, vds
      data = zeros(N,4); 
      % Integrate system for N time steps
      for tt=1:N
        data(tt,:) = [ canon_system.vd canon_system.v canon_system.xd canon_system.x];
        canon_system = canonicalintegratestep(time,dt,canon_system);
      end
      
      % Compare with closed form solution
      [ts xs xds vs vds ] = canonicalintegrate(time,dt,time_exec,order,canon_system.alpha);
      data_cf(:,4) = xs;
      data_cf(:,3) = xds;
      data_cf(:,2) = vs;
      data_cf(:,1) = vds;

      % Plot
      figure(order)
      for dd=1:4
        subplot(2,2,dd);
        plot(t,data(:,dd),'LineWidth',4,'Color',[0.7 0.7 1]);
        hold on
        plot(t,data_cf(:,dd),'--','LineWidth',2,'Color',[0 0 0]);
        hold off
        
        xlabel('t (s)')
        ylabel(labels(dd))
        
        if (dd==1)
          legend('integrated step-by-step','closed form solution')
          title(['System of order ' num2str(order)])
        end
      end

      
    end
  end
end



