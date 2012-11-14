impatient = 1; % 0 -> do everything, 1 -> do less, and thus quicker

% Arm settings
n_dofs = 6;
arm_length = 1;

% Points to reach to
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

%-------------------------------------------------------------------------------
figure(1)
sensitivityanalysis(n_dofs,arm_length,viapoints)


%-------------------------------------------------------------------------------
figure(2)
if (~exist('results_uncertaintyhandling','var'))
  n_experiments = 100;
  if (exist('impatient','var') && impatient)
    n_experiments = 10;
  end
  results_uncertaintyhandling = uncertaintyhandling(n_experiments);
else
  results_uncertaintyhandling = uncertaintyhandling(results_uncertaintyhandling);
end


%-------------------------------------------------------------------------------
if (~exist('learning_histories','var'))
  figure(3)
  n_experiments_per_task = 10;
  n_updates = 20;
  if (exist('impatient','var') && impatient)
    % Do limited number of experiments per task
    n_experiments_per_task = 2;
    % Reduce number of viapoints to 5
    if (n_viapoints>5)
      viapoints = viapoints(round(linspace(1,n_viapoints,5)),:);
      n_viapoints = size(viapoints,1);
    end
  end
  learning_histories = maturationoptimization(viapoints,n_experiments_per_task,n_updates);
end
