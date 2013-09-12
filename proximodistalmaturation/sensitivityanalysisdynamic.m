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

task = task_maturation(viapoints(1,:)',link_lengths_per_arm(arm_type,:));
task_solver = task_maturation_solver(n_dofs);

if (0)
std_costs_all = zeros(n_arm_types,n_viapoints,n_updates,n_dofs+1);

for arm_type = 1:n_arm_types
  figure(arm_type)

  for i_viapoint = 1:n_viapoints
    disp('______________________________________')
    disp(sprintf('arm_type: %d/%d  viapoint: %d/%d',arm_type,n_arm_types,i_viapoint,n_viapoints))

    task.viapoint = viapoints(i_viapoint,:)';


    % For this arm_type and viapoint, compute the average mean and covar over all experiments
    means_per_update = zeros(n_experiments,n_updates,n_dofs,n_basis_functions);
    covars_per_update = zeros(n_experiments,n_updates,n_dofs,n_basis_functions,n_basis_functions);
    for i_experiment = 1:n_experiments
      for i_update = 1:n_updates
        for i_dof = 1:n_dofs
          means_per_update(i_experiment,i_update,i_dof,:) = results_optimization{arm_type,i_viapoint,i_experiment}(i_update).distributions(i_dof).mean;
          covars_per_update(i_experiment,i_update,i_dof,:,:) = results_optimization{arm_type,i_viapoint,i_experiment}(i_update).distributions(i_dof).covar;
        end
      end
    end
    avg_means_per_update = squeeze(mean(means_per_update)); % Size: n_updates X n_dofs X n_basis_functions
    avg_covars_per_update = squeeze(mean(covars_per_update));

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

        %error_ellipse(distributions(1).covar(1:2,1:2),distributions(1).mean(1:2))
        %hold on
        %plot(theta_eps(1,:,1),theta_eps(1,:,2),'.')
        %axis equal
        %pause

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

end

colormap(ones(n_dofs,3))
colormap(jet)
set(0,'DefaultAxesColorOrder',[ colormap; 0 0 0])


arm_labels = {'Human','Equidistant','Inverted Human'};
for arm_type = 1:n_arm_types
  subplot(1,n_arm_types,arm_type)

  std_costs = squeeze(mean(squeeze(std_costs_all(arm_type,:,:,:))));

  %std_costs(:,end) = max(std_costs,[],2);
  for ii=1:n_updates
    std_costs(ii,:) = std_costs(ii,:)./std_costs(ii,end);
  end
  handles = plot(std_costs,'-','LineWidth',2);
  set(handles(end),'LineWidth',3)
  legend('1','2','3','4','5','6','none')
  axis([1 n_updates 0 1.2])
  title(arm_labels(arm_type))
  
  xlabel('Number of updates')
  ylabel('Normalized standard deviation in cost')
  drawnow
end



