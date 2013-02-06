#ifndef FILE_IO_H
#define FILE_IO_H

bool readnumberofrowsandcolumns(char* filename, unsigned& n_rows, unsigned& n_columns);

bool readintfromfile(char* filename, unsigned& output);

bool readcurrentupdate(char* base_directory, unsigned& current_update);

bool readnumberoftrials(char* base_directory, unsigned current_update, unsigned& n_trials);

bool readdmpparameters(char* base_directory, unsigned& current_update, unsigned& n_trials, unsigned& n_dofs, unsigned& n_bases, double***& dmpparameters);

bool readtrajectories(char* base_directory, unsigned& current_update, unsigned& n_trials, unsigned& n_dofs, unsigned& n_timesteps, double****& traj);

bool writecostvars(char* base_directory, unsigned current_update, unsigned n_trials, unsigned n_timesteps, unsigned n_cost_vars, double*** costvars);

#endif
