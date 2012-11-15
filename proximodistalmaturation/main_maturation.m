%-------------------------------------------------------------------------------
% GENERAL SETTINGS 

impatient = 1; % 0 -> do everything, 1 -> do less, and thus quicker
force_redo_experiments = 0; % 0 -> visualize results if already available
export_figures = 0;

% Arm settings
n_dofs = 6;
arm_length = 1;

% Get link lengths
n_arm_types = getlinklengths;
link_lengths_per_arm = zeros(n_arm_types,n_dofs);
for arm_type=1:n_arm_types
  link_lengths_per_arm(arm_type,:) = getlinklengths(arm_type,n_dofs,arm_length);
end

%-------------------------------------------------------------------------------
fprintf('___________________________________________________________________\n')
fprintf('SENSITIVITY ANALYSIS\n')

% Points to reach to for sensitivity analysis (and optimization too)
viapoint_xs =  0.0:0.2:1.0;
viapoint_ys =  0.2:0.2:1.0;
n_viapoints = 0;
clear viapoints;
for viapoint_x=viapoint_xs
  for viapoint_y=viapoint_ys
    viapoint = [viapoint_x viapoint_y]';
    dist_to_shoulder =  sqrt(sum((viapoint).^2));
    if (dist_to_shoulder<=arm_length)
      n_viapoints = n_viapoints + 1;
      viapoints(n_viapoints,:) = viapoint;
    end
  end
end

perturbation_magnitude = 0.1;

figure(1)
sensitivityanalysis(link_lengths_per_arm,perturbation_magnitude,viapoints);

if (export_figures)
  plot2svg('sensitivityanalysis.svg')
end

%-------------------------------------------------------------------------------
fprintf('___________________________________________________________________\n')
fprintf('UNCERTAINTY HANDLING\n')

% Number of experiments for uncertaintly handling
n_experiments_uncertaintyhandling = 100;
if (exist('impatient','var') && impatient)
  n_experiments_uncertaintyhandling = 10;
end

figure(2)
if (force_redo_experiments || ~exist('results_uncertaintyhandling','var') )
  % Do experiments
  results_uncertaintyhandling = uncertaintyhandling(link_lengths_per_arm,viapoints,n_experiments_uncertaintyhandling);
else
  % Visualize experiments
  uncertaintyhandlingvisualize(link_lengths_per_arm,results_uncertaintyhandling);
end

if (export_figures)
  plot2svg('uncertaintyhandling.svg')
end

%-------------------------------------------------------------------------------
fprintf('___________________________________________________________________\n')
fprintf('OPTIMIZATION\n')

% Settings for optimization
n_experiments_per_task = 10;
n_updates = 50;
if (exist('impatient','var') && impatient)
  % Do limited number of updates and experiments per task
  n_updates = 20;
  n_experiments_per_task = 5;
  % Reduce number of viapoints to 5
  if (n_viapoints>5)
    viapoints = viapoints(round(linspace(1,n_viapoints,5)),:);
    n_viapoints = size(viapoints,1);
  end
end

figure(3)
if (force_redo_experiments || ~exist('results_optimization','var') )
  % Do experiments
  results_optimization = maturationoptimization(link_lengths_per_arm,viapoints,n_experiments_per_task,n_updates);
else
  % Visualize experiments
  maturationoptimizationvisualization(link_lengths_per_arm,results_optimization);  
end

if (export_figures)
  plot2svg('optimization.svg')
end


save('proximodistalmaturation.mat','link_lengths_per_arm','perturbation_magnitude','viapoints','results_optimization','results_uncertaintyhandling')
