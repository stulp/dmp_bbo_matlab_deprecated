function write_update_summary(task_name,current_update,update_summary)

parentdir = sprintf('data_%s',task_name);
dirname   = sprintf('%03d_update',current_update);

[status,mess,messid] = mkdir(parentdir,dirname); %#ok<NASGU>

save(sprintf('%s/%s/update_summary.mat',parentdir,dirname),'update_summary');

end