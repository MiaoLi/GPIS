%%%% To run from Matlab_functions

clear all,
close all,

addpath (pwd) ;


% the following line copy the object representation from matlab folder to current folder  
copyfile('../matlab/data_grasp.dat','.');

rootname = 'cylinder_p';


mkdir('Cylinder_mod');
cd './Cylinder_mod'


fcreate_mod_file(rootname);
fAll_mod(rootname);

 cd ..