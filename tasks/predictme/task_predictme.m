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

function [task] = task_predictme(g,y0,g_distract)
if (nargin<1), g   = [0 0]; end
if (nargin<2), y0  = [1 -1]; end
if (nargin<3), g_distract = g + [0.5 0.0]; end

task.name = 'predictme';

task.y0 = y0;
task.g  =  g;
task.g_distract = g_distract;

task.cost_function= @cost_function_predictme;

task.legibility = 1.5;

  function costs = cost_function_predictme(task,cost_vars)

    [n_rollouts n_time_steps n_cost_vars ] = size(cost_vars); %#ok<NASGU>

    plot_me = 0;
    animation = 0;
    g = task.g;
    
    for k=1:n_rollouts
      ys   = squeeze(cost_vars(k,:,1:3:end));
      ydds = squeeze(cost_vars(k,:,3:3:end));

      T = length(ys);

      % Cost due to acceleration
      sum_ydd = sum((sum(ydds.^2,2)));
      cost(k,3) = sum_ydd/100000;

      % Where are the target and the distractor?
      x_target = g(1);
      x_distract = g_distract(1);
      x_between = (x_distract+x_target)/2;

      if (plot_me)
        if (nargin<4)
          color = 0.8*ones(1,3);
        end
        plot(x_target,g(2),'*g')
        plot(x_distract,g(2),'or')
        plot(ys(:,1),ys(:,2),'Color',color)
        axis([-0.1 1.2 -1.1 0.2])
      end

      if (animation)
        figure(11)
        clf
      end

      % Assume the worst: the time the predictor locks onto the target is equal to
      % the total time.
      target_locked_at_time = T;
      locked_count = 0;
      ti=10;
      while (ti<=T)

        %if (ys(ti,2)<0)
        %intersection with y=0
        % Model line as
        % x = x1 + u(x2-x1)
        % y = y1 + u(y2-y1)
        % When y=0 => u = -y1/(y2-y1) => x = x1 + (-y1/(y2-y1))(x2-x1)
        y1 = ys(ti-1,2);
        y2 = ys(ti,2);
        u = -y1/(y2-y1);
        x1 = ys(ti-1,1);
        x2 = ys(ti,1);
        intersection = [ x1 + u*(x2-x1)  0 ];

        % Distance to 0,0
        dist = sqrt(sum(ys(ti,:).^2));
        % Velocity of end-effector
        endeff_vel = sqrt(sum((ys(ti,:)-ys(ti-1,:)).^2));
        ticks_for_dist = dist/endeff_vel;

        % The intersection point and distance define the mean and std of a Gaussian
        % along y=0
        prediction_mean = intersection(1);
        some_scaling_factor = 0.01*task.legibility;
        prediction_sigma = some_scaling_factor*ticks_for_dist;
        % Value of cumululative normal distribution at x_between
        cdf_val = cdf('Normal',x_between,prediction_mean,prediction_sigma);
        % If probability is high enough, consider target to be locked
        if (x_between<x_target)
          cdf_val = 1 - cdf_val;
        end
        if (cdf_val>0.95)
          locked_count = locked_count+1;
        end


        if (animation)
          % Plot target and distractor if necessary
          plot(x_target,g(2),'*g')
          hold on
          plot(x_distract,g(2),'or')
          plot(ys(1:ti,1),ys(1:ti,2),'LineWidth',2)
          axis equal
          axis([-2 2 -2 2])


          plot(intersection(1),intersection(2),'.m')
          plot([x2 intersection(1)],[y2 intersection(2)],'--m')

          % Plot the Gaussian in the plot
          gauss_x=-2:0.01:x_between;
          gauss_y=gaussmf(gauss_x,[prediction_sigma prediction_mean]);
          plot(gauss_x,gauss_y,'-g')
          gauss_x=x_between:0.01:2;
          gauss_y=gaussmf(gauss_x,[prediction_sigma prediction_mean]);
          plot(gauss_x,gauss_y,'-r')


          text(0,1.2,sprintf('Prob target = %1.2f',cdf_val))
          if (locked_count>0)
            text(0,0,sprintf('%d',locked_count))
          end
          hold off
          drawnow
          pause % (0.05)
        end

        %end

        if (locked_count==10)
          target_locked_at_time = ti;
          if (~animation)
            ti=T+1; % Forces loop over time to stop
            % But do continue animation if we are animating
          end
        end

        ti = ti+1;
      end

      % 'target_locked_at_time' is the time the predictor was sure about which target was
      % being approached. Lower values are better, so it is the cost (normalized
      % by the total time so it it [0-1].
      cost(k,2) = target_locked_at_time/T;

      % Total cost is the sum of all the subcomponent costs
      costs(k,1) = sum(cost(k,2:end));

      if (animation)
        fprintf('Cost = %1.2f\n',cost(k,2))
        pause
      end

    end
  end

end

