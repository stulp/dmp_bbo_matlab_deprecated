function [ x ] = getarmpos(angles,link_lengths,ticks,plot_me,color)

if (nargin==0), [ x ] = testgetarmpos; return; end;
if (nargin<2), link_lengths=1; end
if (nargin<3), ticks=[]; end
if (nargin<4), plot_me=0; end
if (nargin<5), color=[0 0 0]; end

[ n_tt n_dofs  ] = size(angles);

% If link_lengths is a scalar, it is interpreted to be the length of an arm with
% links of equal length.
if (isscalar(link_lengths))
  link_lengths = link_lengths*ones(1,n_dofs)/n_dofs;
end
% Check
if (length(link_lengths) ~= n_dofs)
  error('number of links (%d) should be equal to number of dofs (%d)',length(link_lengths),n_dofs);
end

if (isempty(ticks))
  compute_ticks=1:n_tt;
else
  compute_ticks = ticks;
end

% 2D position of links over time
x_links = zeros(n_tt,n_dofs+1,2);

for tt=compute_ticks'
  sum_angles = 0;
  for dof=1:n_dofs
    sum_angles = sum_angles + angles(tt,dof);
    x_links(tt,dof+1,1) = x_links(tt,dof,1) + cos(sum_angles)*link_lengths(dof);
    x_links(tt,dof+1,2) = x_links(tt,dof,2) + sin(sum_angles)*link_lengths(dof);
  end
end

x = squeeze(x_links(ticks,end,:));

% Plotting
if (plot_me)
  % End-effector path
  plot(squeeze(x_links(ticks,end,1)),squeeze(x_links(ticks,end,2)),'k','LineWidth',1,'Color',color)
  hold on
  plot(squeeze(x_links(end,end,1)),squeeze(x_links(end,end,2)),'or')
  
  if (plot_me>1)
    % Bonus plotting
    
    % Links
    plot(squeeze(x_links(:,:,1))',squeeze(x_links(:,:,2))','-','Color',0.7*ones(1,3))
    % Joints
    % remark: don't plot last entry. It is not a joint, but the end-effector
    plot(squeeze(x_links(:,1:end-1,1)),squeeze(x_links(:,1:end-1,2)),'.','Color',0.3*ones(1,3),'MarkerSize',15)
    
    % 'Robot body'
    plot([-0.1 0 0 -0.1 -0.1 0],[0.05 0.05 -0.05 -0.05 0.05 0.05],'-k')
    
    % End-effector path
    plot(squeeze(x_links(ticks,end,1)),squeeze(x_links(ticks,end,2)),'ko-','LineWidth',1)
  end
  hold off
  axis equal
end


  function [x] = testgetarmpos
    % Test this function with some settings
    
    % Arm parameters
    arm_type = 1;
    n_dofs = 10;
    arm_length = 1;
    link_lengths = getlinklengths(arm_type,n_dofs,arm_length);
    
    % Angles over time
    n_tt = 50;
    angles = repmat(linspace(0,pi/n_dofs,n_tt),n_dofs,[])';

    sp = 1;
    for plot_me = 1:2 %#ok<FXUP>
      for every_other_tick = [1 5]
        subplot(2,2,sp); sp = sp+1;
        ticks = 1:every_other_tick:n_tt;
        x = getarmpos(angles,link_lengths,ticks,plot_me);
      end
    end
    
  end

end

