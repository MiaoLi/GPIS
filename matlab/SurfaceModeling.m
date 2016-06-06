    function SurfaceModeling()
    %% This function is used to model the object surface with GP
    %% point cloud filtering + surface modeling + surface uncertainty
    %%%%%%%%
    %%%%%%%%
    %% generate data
        % data = GeneratePointCloud('cylinder',1000,[0.035,0.13]);
        objname = 'teacan_part1';  % possible object names in "obj" folder
        if(exist([objname,'.mat'],'file'))
        delete([objname,'.mat'])
        end
        data = GeneratePointCloud('obj',[],[],objname);
        if(norm(data(1,1:3))>1)
        data(:,1:3)=data(:,1:3)*0.001; % for jug  and spray,pineapple only
        end
        if(size(data,1)>2000)
        data=data(randsample(size(data,1),1000),:);
        end 
        % plot3(data(:,1),data(:,2),data(:,3),'o','color',[0 0.392157 0],'MarkerSize',8, 'MarkerFaceColor',[0 0.392157 0]);hold on;
        % axis equal;
        % %  axis off;
        %  cc

            % data(data(:,3)<-0.1,:)=[]; %jug
            %  plot3(data(:,1),data(:,2),data(:,3),'r.');hold on;
            %  quiver3(data(:,1),data(:,2),data(:,3),data(:,4),data(:,5),data(:,6));hold on
            % cc
            % datadel= data(data(:,1)>0.038,:);
            %  plot_sphere(datadel(:,1:3),0.0015,10,'r')
            % plot_sphere(data(:,1:3),0.0015,10,[0 0.392157 0])
            % axis off;
            % cc

            % plot_sphere(data(:,1:3),0.0015,10,[0 0.392157 0])
            % plot3(data(:,1),data(:,2),data(:,3),'r.');hold on;
            % axis equal;
            % axis off;
            % cc
    data=data(randsample(size(data,1),200),:);
    data(:,1:3) = data(:,1:3)-repmat(mean(data(:,1:3)),size(data(:,1:3),1),1);
    nb=60;
    vthresh=[0.02,0.5];
    varNoise=0.008;
    bPlot=1;
    % 
    [data_output ] = GPFiltering(data, vthresh, varNoise, nb,bPlot);
    plot_sphere(data_output(:,1:3),0.003,20,[0.662745 0.662745 0.662745]);
    axis off;