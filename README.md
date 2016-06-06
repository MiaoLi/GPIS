# GPIS
This is the matlab code for Gaussian Process Implicit Surface.

# Gaussian Process Implicit Surface(GPIS)
## matlab 
Run 'SurfaceModeling.m' to generate the generate the object shape representation using Gaussian Process. The object representation data are saved into 'data_grasp.dat'.

## obj 
include the model of object file, right now we only suppot the obj file format but other format can be adapted too.

##shape-downloads
the script to download more object models online.

#Contact-level Grasp Planning
## ampl- A Mathematical Programming Language [AMPL](https://en.wikipedia.org/wiki/AMPL)

Open 'point_temple.mod' and change 'param datanb :=60;' to the correct number of data points. We will make this automatic later.
Run 'main_cylinder.m' to generate the AMPL file for grasp planning. By default, these files will be generated to folder 'Cylinder_mod'. Copy 'ampl.exe' and 'ipopt.exe' to the generated folder. 
This follwing step is only tested in Windows, in ubuntu, AMPL can not find the ipopt solver, I can not find the reason for this now. If you have a better solution, please let me know.

Run 'ampl.exe', you will see 'ampl:', type 'include All_mod.txt', the solver will try to solve the optimization problem. 
ampl: include All_mod.txt



