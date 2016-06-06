function fcreat_ini_points
% creat the initial points
P1=rand(3,1000)*0.2-0.1;
P2=rand(3,1000)*0.2-0.1;
P3=rand(3,1000)*0.2-0.1;
IniPoint=[P1;P2;P3];

save IniPoint
