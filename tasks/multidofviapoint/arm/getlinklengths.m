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

function link_lengths = getlinklengths(arm_type,n_dofs,arm_length)
% Return the lenghts of the links of an arm
%
% arm_type   - Which type of arm?
%               1) All links have same length
%               2) Approximately human link lengths (6-DOF)
%               3) 'Inverted' human (links the wrong way around)
%
% n_dofs     - Number of links (= number of DOFs) 
%                Default for equidistant links: 10  (armtype==1)
%                Default for human arm: 6
%
% arm_length - Length of the arm
%                Default for equidistant links: 1  (armtype==1)
%                Default for human arm: 0.8 (measured on my own arm ;-)


if (nargin==0)
  % Return number of possible types
  link_lengths = 3;
  return;
end

if (arm_type==1)
  % Set defaults if parameters weren't passed
  if (nargin<2), n_dofs=10; end
  if (nargin<3), arm_length=1; end

  % All links have same size
  link_lengths = arm_length*ones(1,n_dofs)/n_dofs;

else  
  % Set defaults if parameters weren't passed
  if (nargin<2), n_dofs=6; end
  if (nargin<3), arm_length=0.8; end
  
  % Approximate relative human arm lengths
  % http://www.paintdrawpaint.com/2011/01/drawing-basics-proportions-of-arm.html
  rel_link_lengths_human = [1.5 1.25 0.4 0.15 0.15 0.1];
  
  % Convert relative link lengths to absolute values
  link_lengths = arm_length*rel_link_lengths_human/sum(rel_link_lengths_human);
  
  % Choose only first 'n_dofs' degrees of freedom
  link_lengths = link_lengths(1:n_dofs);
  
  if (arm_type==3)
    % This is a very strange human arm
    link_lengths = link_lengths(1,end:-1:1);
  end

end

end