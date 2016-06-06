

function fAll_mod (rootname)

%filename = ['All_mod_',rootname,'.txt'];

 fid  = fopen('All_mod.txt', 'w+');
%fid  = fopen(filename, 'w+');




% %%%% O2
% rootname = 'O2_p';
% orient = '_o';
% extension = '.mod';
% 
% 
% for i=1:42
%     for j=1:3
%        filename = [rootname, num2str(j), orient, num2str(i), extension];
%        fprintf(fid, '%s ;\n','reset');
%        fprintf(fid, '%s %s;\n','model', filename);
%     end
% end

%%%% Cylinder
rootname = 'cylinder_p';
extension = '.mod';
load('../IniPoint.mat');
for i=1:size(IniPoint,2)
       filename = [rootname, num2str(i), extension];
       fprintf(fid, '%s ;\n','reset');
       fprintf(fid, '%s %s;\n','model', filename);
end



fclose(fid);
