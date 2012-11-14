figure(1)
viapoints = sensitivityanalysis;

figure(2)
if (~exist('results_uncertaintyhandling','var'))
  results_uncertaintyhandling = uncertaintyhandling(10);
else
  results_uncertaintyhandling = uncertaintyhandling(results_uncertaintyhandling);
end

if (~exist('learning_history','var'))
  figure(3)
  maturationoptimization;
end
