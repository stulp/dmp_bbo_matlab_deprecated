function [theta_opt learning_history] = evolutionaryoptimization(task,theta_init,covar_init,n_updates,n_samples,update_parameters,varargin)
% Input:
%  task              - task that should be optimized
%  theta_init        - initial parameters
%  covar_init        - initial covariance matrix for exploration
%  n_updates         - number of updates to perform
%  n_samples         - number of roll-outs per update
%  eliteness         - number elite samples per update
%  update_parameters - see "check_update_parameters.m" for the fields it may contain
% Output:

%-------------------------------------------------------------------------------
% Call test function if called without arguments
if (nargin==0)
  [theta_opt learning_history] = testevolutionaryoptimization;
  return
end

%-------------------------------------------------------------------------------
% Process arguments
covar_default = eye(size(theta_init,2));
if (nargin<3);  covar_init            = covar_default; end
if (nargin<4);  n_updates             =            50; end
if (nargin<5);  n_samples             =            10; end

if (nargin<6);  
  % Get default update parameters
  update_parameters     = check_update_parameters; 
  
elseif (~isempty(varargin))
  % Backwards compatibilty. Previous call was like this:
  % evolutionaryoptimization(task,theta_init,covar_init,n_updates,K,eliteness,weighting_method,covar_update,covar_bounds,covar_lowpass,covar_scales)
  eliteness = update_parameters;
  clear update_parameters;
  update_parameters = backwards_compatible_update_parameters(eliteness,varargin);
end

% Sanity check on update parameters
update_parameters     = check_update_parameters(update_parameters);

[ n_dofs n_dims ] = size(theta_init); %#ok<NASGU>
if (ndims(covar_init)==2)
  covar_init = repmat(shiftdim(covar_init,-1),n_dofs,[]);
end
for i_dof=1:n_dofs
  distributions(i_dof).mean  = theta_init(i_dof,:);
  distributions(i_dof).covar = squeeze(covar_init(i_dof,:,:));
end

plot_me = 1;
save_update_summaries = 0;
if (save_update_summaries)
  addpath(genpath('fileio/'))
end

%-------------------------------------------------------------------------------
% Actual optimization loop
i_update = 0;
while (i_update<=n_updates)
  
  %------------------------------------------------------------------
  % Update parameters and sample next batch of thetas
  if (i_update>0)
    [ distributions update_summary ] = update_distributions(distributions,theta_eps,costs,update_parameters);
    learning_history(i_update) = update_summary;
    if (save_update_summaries)
      write_update_summary(task.name,i_update,update_summary)
    end
  end
  
  %------------------------------------------------------------------
  % Sample from distributions
  first_is_mean = 1;
  theta_eps = generate_samples(distributions,n_samples,first_is_mean);
  
  %------------------------------------------------------------------
  % Prepare plotting of roll-outs if necessary
  if (plot_me)
    figure(1)
    if (i_update==0), clf; end
    % Very difficult to see anything in the plots for many dofs
    plot_n_dofs = min(n_dofs,3);
    subplot(plot_n_dofs,4,1:4:plot_n_dofs*4)
    cla
    title('Visualization of roll-outs')
    hold on
  end
  
  %------------------------------------------------------------------
  % Evaluate the last batch of samples 
  costs = task.perform_rollouts(task,theta_eps,plot_me);
  
  
  %------------------------------------------------------------------
  % More plotting
  if (plot_me)
    % Done with plotting of roll-outs
    subplot(n_dofs,4,1:4:n_dofs*4)
    hold off

    if (i_update>0)
      plotlearninghistory(learning_history);
      if (isfield(task,'plotlearninghistorycustom'))
        figure(11)
        task.plotlearninghistorycustom(learning_history)
      end
    end
    %pause; fprintf('Pausing... press key to continue.\n')
    pause(0.1)
  end

  i_update = i_update + 1;
end

save(sprintf('%s_learning_history.mat',task.name),'learning_history');


% Done with optimizing. Return optimal (?) parameters
% These are the means of the distributions for each dof
for i_dof=1:n_dofs
  theta_opt(i_dof,:) = distributions(i_dof).mean;
end

% Main function done
%-------------------------------------------------------------------------------



%-------------------------------------------------------------------------------
% Test function
  function [theta_opt learning_history] = testevolutionaryoptimization

    n_dims = 2;
    
    % Get the task (see nest function '[task] = task_min_dist(target)' below)
    target = zeros(1,n_dims);
    task = task_min_dist(target);

    % Initial parameters
    theta_init = 5*ones(1,n_dims);
    covar_init = 8*eye(n_dims);
    
    % Algorithm parameters
    n_updates = 25;
    n_samples = 15;
    % Update parameters
    update_parameters.weighting_method    = 'PI-BB'; % {'PI-BB','CMA-ES'}
    update_parameters.eliteness           =      10;
    update_parameters.covar_update        = 'PI-BB'; % {'PI-BB','CMA-ES'}
    update_parameters.covar_full          =       0; % 0 -> diag, 1 -> full
    update_parameters.covar_learning_rate =       1; % No lowpass filter
    update_parameters.covar_bounds        =   [0.1]; %#ok<NBRAK> % Lower relative bound
    update_parameters.covar_scales        =       1; % No scaling

    % Run optimization
    clf
    [theta_opt learning_history] = evolutionaryoptimization(task,theta_init,covar_init,n_updates,n_samples,update_parameters); %#ok<NASGU>

    test_backwards_compatibility=0;
    if (test_backwards_compatibility)
      % Test backwards compatibility
      update_parameters.weighting_method    =       2;
      update_parameters.covar_update        =       1;
      [theta_opt learning_history] = evolutionaryoptimization(task,theta_init,covar_init,n_updates,n_samples,...
        update_parameters.eliteness,update_parameters.weighting_method,update_parameters.covar_update,update_parameters.covar_bounds,update_parameters.covar_learning_rate,update_parameters.covar_scales);
    end
    
    
    % Here is an example of how to design a task. This one simply returns the
    % distance to the dist.
    function [task] = task_min_dist(target)

      task.name = 'min_dist';
      task.perform_rollouts = @perform_rollouts_min_dist;
      task.target = target;
      task.n_dims = length(target);

      % Now comes the function that does the roll-out and visualization thereof
      function costs = perform_rollouts_min_dist(task,thetas,plot_me,color)
        thetas = squeeze(thetas); % Remove first dummy dimension

        % Cost is distance to target
        n_samples = size(thetas,1);
        target_rep = repmat(target,n_samples,1);        
        costs = sqrt(sum((thetas-target_rep).^2,2));
        
        % Plot if necessary
        if (nargin>2 && plot_me)
          if (nargin<4)
            color = [0 0 0.6];
          end
          if (task.n_dims==2)
            for k=1:n_samples
              plot([target_rep(k,1) thetas(k,1)],[target_rep(k,2) thetas(k,2)],'-','Color',color)
            end
            plot(thetas(1,1),thetas(1,2),'o','Color',0.5*color)
            plot(thetas(2:end,1),thetas(2:end,2),'o','Color',color)
            axis equal
            axis([-10 10 -10 10])
          elseif (task.n_dims==3)
            plot3([target_rep(:,1) thetas(:,1)],[target_rep(:,2) thetas(:,2)],[target_rep(:,3) thetas(:,3)],'-','Color',color)
            plot3(thetas(:,1),thetas(:,2),thetas(:,3),'o','Color',color)
            axis equal
            axis([-10 10 -10 10 -10 10])
            view(45,45)
          end
        end

      end

    end

  end

end