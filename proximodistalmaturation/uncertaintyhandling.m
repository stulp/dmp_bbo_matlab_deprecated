function results = uncertaintyhandling(varargin)

% Default: run some new experiments
run_new_experiments = 1;
n_experiments = 100;

if (~isempty(varargin))
  if (isscalar(varargin{1}))
    % Argument determines how many experiments to run
    n_experiments = varargin{1};
  else
    % Experiments have already been done. No need to run new ones.
    results = varargin{1};
    run_new_experiments = 0;
  end
end

n_dofs = 6;
arm_length = 1;

n_arm_types = getlinklengths;

if (run_new_experiments)

  % For each combination of joints:
  % sample perturbation of both from a Gaussian
  % determine four costs
  % A 1 perturbed & 2 perturbed
  % B 1 perturbed, 2 not
  % C 1 not, 2 perturbed
  % D 1 not, 2 not
  % How often does 2 change the rank?
  % i.e. how often does C or D lie between A-B

  for arm_type=1:n_arm_types
    link_lengths = getlinklengths(arm_type,n_dofs,arm_length);
    results{arm_type} = nan*ones(n_dofs,n_dofs);


    plot_me=0;
    viapoint_x = 0.5;
    viapoint_y = 0.5;

    for which_angle_1=1:(n_dofs-1)
      for which_angle_2=(which_angle_1+1):n_dofs
        fprintf('%d %d ',which_angle_1,which_angle_2);

        results{arm_type}(which_angle_1,which_angle_2) = 0;

        for experiments=1:n_experiments
          if (mod(experiments,10)==1)
            fprintf('*')
          end
          clf
          cur_results = zeros(2,2);
          for angle_1_on_off=0:1
            angle_1 = pi/10*randn;
            for angle_2_on_off=0:1 % angle_1_on_off:1 % 0:1
              angle_2 = pi/10*randn;
              angles = zeros(1,n_dofs);
              %angles(which_angle_1) = angle_1_on_off*angle_1;
              %angles(which_angle_2) = angle_2_on_off*angle_2;
              angles(which_angle_1) = angle_1;
              angles(which_angle_2) = angle_2;
              hold on
              x = getarmpos(angles,link_lengths,1,plot_me);
              viapoint = [viapoint_x viapoint_y]';
              if (plot_me)
                hold on
                plot(viapoint_x,viapoint_y,'o')
              end
              cur_results(angle_1_on_off+1,angle_2_on_off+1) = sqrt(sum((x-viapoint).^2));
            end
          end
          %fprintf(' off/off  off/on\n ON/OFF   on/on')
          %cur_results
          if (cur_results(1,1)<cur_results(2,1) && cur_results(1,2)<cur_results(2,2))
            fprintf('D')
            results{arm_type}(which_angle_1,which_angle_2) = results{arm_type}(which_angle_1,which_angle_2) + 1;
          elseif (cur_results(1,1)>cur_results(2,1) && cur_results(1,2)>cur_results(2,2))
            fprintf('D')
            results{arm_type}(which_angle_1,which_angle_2) = results{arm_type}(which_angle_1,which_angle_2) + 1;
          end

          if (plot_me)
            pause(1)
          end
        end
        fprintf('\n')
      end
    end
    results{arm_type} = results{arm_type}/n_experiments;
  end
end

clf
for arm_type=1:n_arm_types
  link_lengths = getlinklengths(arm_type,n_dofs,arm_length);

  subplot(2,n_arm_types,arm_type)
  getarmpos(0.0*ones(1,n_dofs),link_lengths,1,2);
  xlim([-0.1 1.1])

  subplot(2,n_arm_types,n_arm_types+arm_type)
  h = bar3(results{arm_type});
  %h = bar3(results{arm_type}(1:n_dofs-1,2:n_dofs));
  remove_empty_bars(h);
  axis equal
  axis square;
  xlabel('distal joint');
  ylabel('proximal joint');
  zlabel('% that distal joint doesnt matter')
  set(gca,'YTick',1:n_dofs-1)
  set(gca,'XTick',2:n_dofs)
  view(-45,20);
  zlim([0 1])
  colormap(gray)
  drawnow
end


  function remove_empty_bars(hBars)
    % Taken from: http://stackoverflow.com/questions/2050367/how-to-hide-zero-values-in-bar3-plot-in-matlab
    for iSeries = 1:numel(hBars)
      zData = get(hBars(iSeries),'ZData');  %# Get the z data
      index = logical(kron(zData(2:6:end,2) == 0,ones(6,1)));  %# Find empty bars
      zData(index,:) = nan;                 %# Set the z data for empty bars to nan
      set(hBars(iSeries),'ZData',zData);    %# Update the graphics objects
    end
  end

end
