#include <fstream>
#include <string>

#include "fileio.h"

int main(int n_args, char** args)
{
  bool verbose = false;
  
  char base_directory[512] = "./data";
  if (n_args>1) {
    sprintf(base_directory,"%s",args[1]);
  }

  unsigned current_update, n_trials, n_dofs, n_timesteps, n_bases;

  //-----------------------------------------------------------------------------------------------  
  // Read trajectories and dmp parameters
  double**** trajectories = NULL;
  if (!readtrajectories(base_directory, current_update, n_trials, n_dofs, n_timesteps, trajectories))
    return -1;

  //double*** thetas = NULL;
  //if (!readdmpparameters(base_directory, current_update, n_trials, n_dofs, n_bases, thetas))
  //  return -1;
  
  if (verbose) {
    printf("  current_update   = %d\n",current_update);
    printf("  n_trials         = %d\n",n_trials);
    printf("  n_dofs           = %d\n",n_dofs);
    printf("  n_bases          = %d\n",n_bases);
    printf("  n_timesteps      = %d\n",n_timesteps);
  }  

  //-----------------------------------------------------------------------------------------------  
  // Determine the cost-relevant variables from the trajectories
  
  // Save back in cost vars
  unsigned n_cost_vars = n_dofs*3; 
  // Initialize array
  double*** cost_vars = new double**[n_trials];
  for (int i_trials=0; i_trials<n_trials; i_trials++) {
    cost_vars[i_trials] = new double*[n_timesteps];
    for (int i_timestep=0; i_timestep<n_timesteps; i_timestep++) {
      cost_vars[i_trials][i_timestep] = new double[n_cost_vars];
    }
  }

  //printf("External program executing trial: ");
  for (int i_trials=0; i_trials<n_trials; i_trials++)
  {
    printf("%d/%d  ",i_trials+1,n_trials);

    for (int i_timestep=0; i_timestep<n_timesteps; i_timestep++)
    {
      for (int i_dof=0; i_dof<n_dofs; i_dof++)
      {
        cost_vars[i_trials][i_timestep][3*i_dof+0] = trajectories[i_trials][i_dof][i_timestep][0]; // Position
        cost_vars[i_trials][i_timestep][3*i_dof+1] = trajectories[i_trials][i_dof][i_timestep][1]; // Velocity
        cost_vars[i_trials][i_timestep][3*i_dof+2] = trajectories[i_trials][i_dof][i_timestep][2]; // Acceleration
      }
    }
  }
  //printf("\n");


  
  //-----------------------------------------------------------------------------------------------  
  // Write the cost-relevant variables to file
  if (!writecostvars(base_directory, current_update, n_trials, n_timesteps, n_cost_vars, cost_vars))
    return -1;
  
  return 0;
}

