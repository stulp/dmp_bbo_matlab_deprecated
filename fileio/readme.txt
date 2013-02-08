________________________________________________________________________________
RELEVANT FILES

basename/                   Whichever you choose

  current_update.txt        Contains one scalar number, representing the current update

  001_update/               Directory with files for the 1st update

    update_summary.mat      Matlab variables summarizing the update

    rollouts/               Rollouts as txt files
      number_of_trials.txt  Contains scalar: number of trials to execute for this update  
  
      01_dmpparameters.txt  The DMP weights. Size: n_dofs x n_bases
  
      01_traj_x.txt         Trajectory position. Size: n_timesteps x n_dofs
      01_traj_xd.txt        Trajectory velocity. Size: n_timesteps x n_dofs
      01_traj_xdd.txt       Trajectory acceleration. Size: n_timesteps x n_dofs
  
      01_costvars.txt       Cost relevant variables. Size: n_timesteps x n_costvars
                            Number of variables depends on task.
  
      02_*                  As above, but for second trial
      etc.                  Up to the number in number_of_trials.txt

    
  002_update/      Directory with files for the 2nd update
  etc.

________________________________________________________________________________
IDEA

  Matlab writes current_update.txt, number_of_trials.txt, dd_dmpparameters.txt and/or dd_traj_x*.txt
    This writing should be done with "write_output_to_ascii.m"

  External program reads them and writes dd_costvars.txt
    For C++ this can be done with fileio.h/libfileio.a

________________________________________________________________________________
EXAMPLES

Example of reading/writing these files can be found in
  task/task_viapoint.m
  task/task_viapoint_external_cpp/task_viapoint_external_cpp.cpp