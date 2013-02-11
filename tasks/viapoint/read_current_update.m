function current_update = read_current_update(directory)
filename = sprintf('%s/current_update.txt',directory);
if (~exist(filename,'file'))
  current_update = 1;
  dlmwrite(filename,current_update,' ');
else
  current_update = load(filename);
end
end