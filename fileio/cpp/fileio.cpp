#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <algorithm>    // std::count

#include "fileio.h"

using namespace std;

bool readnumberofrowsandcolumns(char* filename, unsigned& n_rows, unsigned& n_columns) {
  // http://stackoverflow.com/questions/3482064/counting-the-number-of-lines-in-a-text-file

  // Open file
  std::ifstream myfile;
  myfile.open(filename);
  if (myfile.fail()) {
    printf("ERROR: Could not open %s. Abort.\n",filename);
    return false;
  } 
  
  // Read first line to determine number of columns
  char line[2048];
  myfile.getline(line,2048);
  stringstream token_stream(line); 
  unsigned n_tokens = 0;
  double dummy;
  while (token_stream >> dummy) {
    n_tokens++;
    //std::cout << dummy << " " << n_columns << std::endl;
  }
  
  // Read number of lines
  // new lines will be skipped unless we stop it from happening:    
  myfile.unsetf(std::ios_base::skipws);
  // count the newlines with an algorithm specialized for counting:
  unsigned n_lines = std::count(
      std::istream_iterator<char>(myfile),
      std::istream_iterator<char>(), 
      '\n');
  
  myfile.close();

  n_lines++; // We had already read the first line at the beginning!
  

  n_columns = n_tokens;
  n_rows    = n_lines;
  
  return true;
}

bool readintfromfile(char* filename, unsigned& output) {
  fstream filestr;
  filestr.open(filename, fstream::in );
  if (filestr.fail()) {
    printf("ERROR: Could not open %s. Abort.\n",filename);
    output = 0;
    return false;
  }
  filestr >> output;
  filestr.close();
  
  return true;
}

bool readcurrentupdate(char* base_directory, unsigned& current_update) {
  char filename[512];
  sprintf(filename,"%s/current_update.txt",base_directory);
  return readintfromfile(filename, current_update);
}

bool readnumberoftrials(char* base_directory, unsigned current_update, unsigned& n_trials) {
  char filename[512];
  sprintf(filename,"%s/%03d_update/rollouts/number_of_trials.txt",base_directory,current_update);
  return readintfromfile(filename, n_trials);
}


bool readdmpparameters(char* base_directory, unsigned& current_update, unsigned& n_trials, unsigned& n_dofs, unsigned& n_bases, double***& dmpparameters) {

  bool verbose = false;
  
  if (!readcurrentupdate(base_directory, current_update))
    return false;
  if (!readnumberoftrials(base_directory, current_update, n_trials))
    return false;

  char filename[512];
  char outputdir[512];
  sprintf(outputdir,"%s/%03d_update/rollouts",base_directory,current_update);
  sprintf(filename,"%s/%02d_dmpparameters.txt",outputdir,1);
  if (!readnumberofrowsandcolumns(filename,n_dofs,n_bases)) 
    return false;
  
  // Initialize array
  dmpparameters = new double**[n_trials];
  for (int i_trials=0; i_trials<n_trials; i_trials++) {
    dmpparameters[i_trials] = new double*[n_dofs];
    for (int i_dof=0; i_dof<n_dofs; i_dof++) {
      dmpparameters[i_trials][i_dof] = new double[n_bases];
    }
  }
  
  // Read trajectories from file   
  fstream filestr;
  for (int i_trials=0; i_trials<n_trials; i_trials++)
  {
    sprintf(filename,"%s/%02d_dmpparameters.txt",outputdir,i_trials+1);
    if (verbose) printf("%s\n",filename);
    
    filestr.open(filename, fstream::in );
    if (filestr.fail()) {
      printf("ERROR: Could not open %s. Abort.\n",filename);
      return -1;
    }

    for (int i_bases=0; i_bases<n_bases; i_bases++) {
      for (int i_dof=0; i_dof<n_dofs; i_dof++) {
        filestr >> dmpparameters[i_trials][i_dof][i_bases];
        if (verbose) printf("%1.3f ",dmpparameters[i_trials][i_dof][i_bases]);
      }
    }
    if (verbose) printf("\n"); 

    filestr.close();
    
  }

  return true;  
}



bool readtrajectories(char* base_directory, unsigned& current_update, unsigned& n_trials, unsigned& n_dofs, unsigned& n_timesteps, double****& traj) {

  bool verbose = false;

  if (!readcurrentupdate(base_directory, current_update))
    return false;
  if (!readnumberoftrials(base_directory, current_update, n_trials))
    return false;

  char filename[512];
  char outputdir[512];
  sprintf(outputdir,"%s/%03d_update/rollouts",base_directory,current_update);
  sprintf(filename,"%s/%02d_traj_x.txt",outputdir,1);
  if (!readnumberofrowsandcolumns(filename,n_timesteps,n_dofs)) 
    return false;

  if (verbose) {
    printf("  current_update   = %d\n",current_update);
    printf("  n_trials         = %d\n",n_trials);
    printf("  n_dofs           = %d\n",n_dofs);
    printf("  n_timesteps      = %d\n",n_timesteps);
  }  
 
  // Initialize array
  traj = new double***[n_trials];
  for (int i_trials=0; i_trials<n_trials; i_trials++) {
    traj[i_trials] = new double**[n_dofs];
    //traj = new int[n_trials][n_dofs][n_timesteps][3];
    for (int i_dof=0; i_dof<n_dofs; i_dof++) {
      traj[i_trials][i_dof] = new double*[n_timesteps];
      for (int i_timestep=0; i_timestep<n_timesteps; i_timestep++) {
        traj[i_trials][i_dof][i_timestep] = new double[3];
      }
    }
  }
  
  // Read trajectories from file   
  fstream filestr;
  for (int i_trials=0; i_trials<n_trials; i_trials++)
  {
    for (int pos_vel_acc=0; pos_vel_acc<3; pos_vel_acc++) {

      sprintf(filename,"%s/%02d_traj_x",outputdir,i_trials+1);
      // Add 'd's to get velocity or accelerations
      for (int dd=0; dd<pos_vel_acc; dd++) {
        sprintf(filename,"%sd",filename);        
      }
      sprintf(filename,"%s.txt",filename);        
      if (verbose) printf("%s\n",filename);
      
      filestr.open(filename, fstream::in );
      if (filestr.fail()) {
        printf("ERROR: Could not open %s. Abort.\n",filename);
        return -1;
      }

      for (int i_timestep=0; i_timestep<n_timesteps; i_timestep++) {
        for (int i_dof=0; i_dof<n_dofs; i_dof++) {
          filestr >> traj[i_trials][i_dof][i_timestep][pos_vel_acc];
          if (verbose) printf("%1.3f ",traj[i_trials][i_dof][i_timestep][pos_vel_acc]);
        }
      }
      if (verbose) printf("\n"); 

      filestr.close();
      
    }
  }

  return true;
}

bool writecostvars(char* base_directory, unsigned current_update, unsigned n_trials, unsigned n_timesteps, unsigned n_cost_vars, double*** costvars)  {
 
  
  char filename[512];
  char outputdir[512];
  sprintf(outputdir,"%s/%03d_update/rollouts",base_directory,current_update);
  
  fstream filestr;
  for (int i_trials=0; i_trials<n_trials; i_trials++)
  {
    sprintf(filename,"%s/%02d_costvars.txt",outputdir,i_trials+1);
    filestr.open(filename, fstream::out );
    if (filestr.fail()) {
      printf("ERROR: Could not open %s. Abort.\n",filename);
      return -1;
    }
    
    for (int i_timestep=0; i_timestep<n_timesteps; i_timestep++)
    {
      for (int var=0; var<n_cost_vars; var++)
      {
        if (var>0)
          filestr << " "; 
        filestr << costvars[i_trials][i_timestep][var];
      }
      filestr << "\n"; 
    }
    filestr.close();
  }

  sprintf(filename,"%s/done.txt",base_directory);
  filestr.open(filename, fstream::out );
  if (filestr.fail()) {
    printf("ERROR: Could not open %s. Abort.\n",filename);
    return -1;
  }
  filestr << "Done!\n";
  filestr.close();
  
  return true;
}
