%--------------- Startup ---------------
disp('startup dmp_bbo')
addpath(genpath('dynamicmovementprimitive'))
addpath(genpath('evolutionaryoptimization'))
addpath(genpath('fileio'))
addpath(genpath('tasks'))
if (exist('dealiasing','dir'))
  addpath(genpath('dealiasing'))
end
if (exist('proximodistalmaturation','dir'))
  addpath(genpath('proximodistalmaturation'))
end
