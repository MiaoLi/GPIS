function [DataOut] = GeneratePointCloud(name,nb,size,objname)
%% This function is used to generate object point cloud
%% name: string, the name of object;
%% nb: how many points will be generated;
%% size: the size of the object, the height and the radius.
switch name
    case 'cylinder'
        %size: radius, height
        nbBottom =ceil(nb/5);
        nbSideOut=nb-ceil(nb/5)*2;
        nbTop = ceil(nb/5);
        Rout=size(1);
        r=Rout*(rand(nbBottom,1)+rand(nbBottom,1));
        for i=1:nbBottom
            if r(i)>Rout
                r(i)=Rout*2-r(i);
            end
        end
        theta=2*pi*rand(nbBottom,1);
        XBottom = [r.*cos(theta),r.*sin(theta),-size(2)/2*ones(nbBottom,1)];
        YBottom =repmat([0,0,-1],nbBottom,1);
        r=Rout*(rand(nbTop,1)+rand(nbTop,1));
        for i=1:nbTop
            if r(i)>Rout
                r(i)=Rout*2-r(i);
            end
        end
        theta=2*pi*rand(nbTop,1);
        XTop = [r.*cos(theta),r.*sin(theta),size(2)/2*ones(nbTop,1)];
        YTop =repmat([0,0,1],nbTop,1);
        
        theta=2*pi*rand(nbSideOut,1);
        XSideOut = [Rout*cos(theta),Rout*sin(theta),size(2)*rand(nbSideOut,1)-size(2)/2];
        YSideOut = zeros(nbSideOut,3);
        for i=1:nbSideOut
            YSideOut(i,:)=[cos(theta(i)),sin(theta(i)),0];
        end
        XData=[XBottom;XSideOut;XTop;];
        YData=[YBottom;YSideOut;YTop;];
        DataOut = [XData,YData];
    case 'obj'
        filename=['../obj/',objname,'.obj'];
        if(exist([objname,'.mat'],'file'))
            DataOut=load([objname,'.mat']);
            DataOut =DataOut.data;
        else
            [V,F] = read_vertices_and_faces_from_obj_file(filename);
%           V=[1 0 0;0 0 1; 0 -1 0]'*V';  % change the y and z axis
%           V=V';
            data = convert_vf_to_data( V, F );
            DataOut = data'; %N x D
            save([objname,'.mat'],'data');
        end
%          plot_obj(filename);
    otherwise
        warning('wrong name to generate object point cloud')
end

end