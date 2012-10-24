A task is a structure with 

1) task parameters (e.g. where is the viapoint)

2) a "perform_rollout" function which executes the task for a particular parameter vector theta
Functions for doing black-box optimization of tasks This function should have the signature
  cost = perform_rollout(task,theta,plot_me,color)
where 
   task    - the task itself (to have access to its parameters), 
   theta   - the parameter vector with which the rollout is performed
   plot_me - wether to visualize the rollout (default: do not plot)
   color   - color of the visualizer rollout (handy to set from outside)