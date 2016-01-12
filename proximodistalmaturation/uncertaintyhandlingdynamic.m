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

[n_arm_types n_viapoints n_experiments]  = size(results_optimization); %#ok<USENS>
n_dofs = length(results_optimization{1,1,1}(1).distributions);
n_basis_functions = length(results_optimization{1,1,1}(1).distributions(1).mean);
n_updates = length(results_optimization{1,1,1});

task = task_maturation(viapoints(1,:)',link_lengths_per_arm(arm_type,:));
task_solver = task_maturation_solver(n_dofs);

use_n_updates = [1 n_updates];
for i_update = 1:length(use_n_updates)
  use_update = use_n_updates(i_update);

  for arm_type = 1:n_arm_types

    for i_viapoint = 1:n_viapoints
      disp('______________________________________')
      fprintf('update: %d  arm_type: %d/%d  viapoint: %d/%d\n',use_update,arm_type,n_arm_types,i_viapoint,n_viapoints)
      
      task.viapoint = viapoints(i_viapoint,:)';
      
      % For this update, arm_type and viapoint, compute the average mean and covar over all experiments
      means_per_update = zeros(n_experiments,n_dofs,n_basis_functions);
      covars_per_update = zeros(n_experiments,n_dofs,n_basis_functions,n_basis_functions);
      for i_experiment = 1:n_experiments
        for i_dof = 1:n_dofs
          means_per_update(i_experiment,i_dof,:) = results_optimization{arm_type,i_viapoint,i_experiment}(i_update).distributions(i_dof).mean;
          covars_per_update(i_experiment,i_dof,:,:) = results_optimization{arm_type,i_viapoint,i_experiment}(i_update).distributions(i_dof).covar;
        end
      end
      avg_means_per_update = squeeze(mean(means_per_update)); % Size: n_dofs X n_basis_functions
      avg_covars_per_update = squeeze(mean(covars_per_update));
      
      for which_angle_1=1:(n_dofs-1) % proximal
        for which_angle_2=(which_angle_1+1):n_dofs % distal
          fprintf('arm_type = %d/%d  proximal=%d/%d distal=%d/%d ',arm_type,n_arm_types,which_angle_1,n_dofs-1,which_angle_2,n_dofs);
          
          results{arm_type}(which_angle_1,which_angle_2) = 0;
          
          % Put into the distributions structure
          for i_dof = 1:n_dofs
            distributions(i_dof).mean  = squeeze(avg_means_per_update(i_dof,:));
            distributions(i_dof).covar = squeeze(avg_covars_per_update(i_dof,:,:));
          end
          
          %------------------------------------------------------------------
          % Sample from distributions
          first_is_mean = 0;
          n_samples = 100;
          theta_eps = generate_samples(distributions,n_samples,first_is_mean);
        end
      end
    end



    % Do rollouts for these distributions
    n_samples = 100;
    all_costs = 0.05*ones(n_updates,n_dofs+1,n_samples);
    for i_update = 1:n_updates
      i_update
      for i_dof_frozen = 1:n_dofs+1

        % Put into the distributions structure
        for i_dof = 1:n_dofs
          distributions(i_dof).mean  = squeeze(avg_means_per_update(i_update,i_dof,:))';
          distributions(i_dof).covar = squeeze(avg_covars_per_update(i_update,i_dof,:,:));
          if (i_dof_frozen==i_dof)
            distributions(i_dof).covar(:,:) = 0;
          end
        end

        %------------------------------------------------------------------
        % Sample from distributions
        first_is_mean = 0;
        theta_eps = generate_samples(distributions,n_samples,first_is_mean);

        error_ellipse(distributions(1).covar(1:2,1:2),distributions(1).mean(1:2))
        hold on
        plot(theta_eps(1,:,1),theta_eps(1,:,2),'.')
        axis equal
        pause

        %------------------------------------------------------------------
        % Perform rollouts for the samples in theta_eps
        cost_vars = task_solver.perform_rollouts(task,theta_eps);

        %------------------------------------------------------------------
        % Evaluate the last batch of rollouts
        costs = task.cost_function(task,cost_vars);

        all_costs(i_update,i_dof_frozen,:) = costs(:,1);
      end

    end
      clf
      colors = get(0,'DefaultAxesColorOrder');
      avg_costs = mean(all_costs,3);
      std_costs = std(all_costs,1,3);

      %std_costs(:,end) = max(std_costs,[],2);
      %for ii=1:n_updates
      %  std_costs(ii,:) = std_costs(ii,:)./std_costs(ii,end);
      %end
      plot(std_costs,'-o')
      legend('1','2','3','4','5','6','none')
      %axis([1 n_updates 0 1.3])
      drawnow

      std_costs_all(arm_type,i_viapoint,:,:) = std_costs;
  end
end

% For each combination of joints:
% sample perturbation of both from a Gaussian
% determine four costs
% A 1 perturbed & 2 perturbed
% B 1 perturbed, 2 not
% C 1 not, 2 perturbed
% D 1 not, 2 not
% How often does 2 change the rank?
% i.e. how often does C or D lie between A-B

n_viapoints = size(viapoints,1);

n_arm_types = size(link_lengths_per_arm,1);
for arm_type=1:n_arm_types
  link_lengths = link_lengths_per_arm(arm_type,:);
  n_dofs = length(link_lengths);

  results{arm_type} = nan*ones(n_dofs,n_dofs);

  plot_me=0;

  for which_angle_1=1:(n_dofs-1)
    for which_angle_2=(which_angle_1+1):n_dofs
      fprintf('arm_type = %d/%d  proximal=%d/%d distal=%d/%d ',arm_type,n_arm_types,which_angle_1,n_dofs-1,which_angle_2,n_dofs);

      results{arm_type}(which_angle_1,which_angle_2) = 0;

      for i_viapoint=1:n_viapoints
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
              viapoint = viapoints(i_viapoint,:)';
              if (plot_me)
                hold on
                plot(viapoint(1),viapoint(2),'o')
              end
              cur_results(angle_1_on_off+1,angle_2_on_off+1) = sqrt(sum((x-viapoint).^2));
            end
          end
          %fprintf(' off/off  off/on\n ON/OFF   on/on')
          %cur_results
          if (cur_results(1,1)<cur_results(2,1) && cur_results(1,2)<cur_results(2,2))
            results{arm_type}(which_angle_1,which_angle_2) = results{arm_type}(which_angle_1,which_angle_2) + 1;
          elseif (cur_results(1,1)>cur_results(2,1) && cur_results(1,2)>cur_results(2,2))
            results{arm_type}(which_angle_1,which_angle_2) = results{arm_type}(which_angle_1,which_angle_2) + 1;
          end

          if (plot_me)
            axis equal
            axis([-0.5 1.1 -0.5 1.1])
            pause
          end
        end
      end
      fprintf('\n')
    end
  end
  results{arm_type} = results{arm_type}/(n_viapoints*n_experiments);

  % Plot results
  uncertaintyhandlingvisualize(link_lengths_per_arm,results);
  drawnow
end

