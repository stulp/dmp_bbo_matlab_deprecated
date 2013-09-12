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

function results = uncertaintyhandling(link_lengths_per_arm,viapoints,n_experiments)

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



end
