function fcreate_mod_file(rootname)

%shape: parameters for the object

extension = '.mod';

% Initial positions
load IniPoint;

for i=1:size(IniPoint,2)
    
    filename = [rootname, num2str(i), extension];
    sol_filename = ['sol_',rootname, num2str(i),'.txt'];
    
    fid  = fopen(filename, 'w+');
    fid2 = fopen('../point_temple.mod','r');
    
    while(~feof(fid2))
        line = fgetl(fid2);
%         if(strfind(line, 'param datanb :='))
%             fprintf(fid, '%s %f ;\n',line, IniPoint(1,i));
%         end
        
        if ( strfind(line, 'let p[1,1] :=') )
            fprintf(fid, '%s %f ;\n',line, IniPoint(1,i));
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(2,i));
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(3,i))
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(4,i));
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(5,i));
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(6,i));
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(7,i));
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(8,i));
            line = fgetl(fid2);
            fprintf(fid, '%s %f ;\n',line, IniPoint(9,i));
            
        else if ( strfind(line, 'display solve_message') )
                fprintf(fid, '%s %s  %s ;\n',line, '>>', sol_filename);
                for k=1:8
                    line = fgetl(fid2);
                    fprintf(fid, '%s %s  %s ;\n',line, '>>', sol_filename);
                end
            else
                fprintf(fid, '%s \n',line);
            end
        end
    end
    
    
    fclose(fid);
    fclose(fid2);
    
end





