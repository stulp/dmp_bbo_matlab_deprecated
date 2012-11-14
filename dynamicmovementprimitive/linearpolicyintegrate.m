function [trajectory] = linearpolicyintegrate(activations,theta,dt,figure_handle)

if (nargin==0), [trajectory] = testlinearpolicyintegrate; return; end;
if (nargin<4), figure_handle=0; end;

n_dim = size(theta,1);
n_timesteps = size(activations,1);

ydd = zeros(n_timesteps,n_dim);
yd = ydd;
y = ydd;
ts = dt*(0:n_timesteps-1)';

for i_dim=1:n_dim
  % Compute acceleration from basis functions
  ydd(:,i_dim) = sum(repmat(theta(i_dim,:),n_timesteps,1).*activations,2);

  % Integrate to get velocity and position over time.
  for tt=2:n_timesteps
    yd(tt,i_dim)  = yd(tt-1,i_dim) + dt*ydd(tt,i_dim);
    y(tt,i_dim)   = y(tt-1,i_dim)  + dt*yd(tt,i_dim);
  end
end


trajectory.ts = ts;
trajectory.y = y;
trajectory.yd = yd;
trajectory.ydd = ydd;


if (figure_handle)
  figure(figure_handle)
  for i_dim=1:n_dim


    subplot(n_dim,4,(i_dim-1)*4+1)
    plot(ts,activations)
    hold on
    % Little trick to get the centers of the basis functions: center is their mode
    [values indices] = max(activations);
    stem(ts(indices),theta(i_dim,:))
    hold off
    xlabel('time')
    ylabel('activation')

    subplot(n_dim,4,(i_dim-1)*4+2)
    plot(ts,ydd(i_dim,:))
    xlabel('time')
    ylabel('ydd')

    subplot(n_dim,4,(i_dim-1)*4+3)
    plot(ts,yd(i_dim,:))
    xlabel('time')
    ylabel('yd')

    subplot(n_dim,4,(i_dim-1)*4+4)
    plot(ts,y(i_dim,:))
    xlabel('time')
    ylabel('y')
    
  end
end

  function [trajectory] = testlinearpolicyintegrate
    % Initialize the basis functions once
    n_basis_functions = 10;
    time = 2;
    dt = 1/100;
    n_timesteps = ceil(time/dt);
    ts = dt*(0:n_timesteps-1)';
    widths = (0.4*time/n_basis_functions)*ones(1,n_basis_functions);
    centers = linspace(4*widths(1),time-4*widths(1),n_basis_functions);
    activations = basisfunctionactivations(centers,widths,ts);

    theta = randn(2,n_basis_functions);

    figure_handle = 1;
    trajectory = linearpolicyintegrate(activations,theta,dt,figure_handle);

  end

end