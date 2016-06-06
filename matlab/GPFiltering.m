function [data_output,data_output_cur] = GPFiltering(data, vthresh, varNoise, nb,bPlot)
%% This function can be used to fliter point cloud;
%% Miao Li,LASA@EPFL,20.04.2014

%% Input:
%% data: N x 6;
%% vthresh: 2 x 1, [value_thresh, normal_thresh];
%% varNoise: noise level;
%% nb:  the number of datapoints in the active data set;
%% bPlot: flag to plt estimated surface or not;

%% data_output: N x 6;
%% data_output_cur: data points from large curvature during the whole filtering process;
%%
% preprocess the data;
[data,inputmean,inputscale] = center_and_normalize_data(data);
R = max(pdist(data(:,1:3),'euclidean')); % kernel parameter

nbTrainingData=1;
idx=randsample(size(data,1),nbTrainingData);
dataon=data(idx,:);
xon=dataon(:,1:3);
ytemp=dataon(:,4:6);
for i=1:size(xon,1)
    yon(4*i-3:4*i,1)=[0, ytemp(i,:)]';
end
xin=[0,0,0;];
yin=[-1 0 0 0]';

n=20;
thetar=pi*rand(1,n);
phi=2*pi*rand(1,n);
[x,y,z]=sph2cart(thetar,phi,1.1*ones(1,n)+0.1*rand(1,n));
xout=[x;y;z]';
for i=1:size(xout,1)
    yout(4*i-3:4*i,1)=[1, 0 0 0]';
end
BVX=[xin;xout;xon;];
BVY=[yin;yout;yon;];

% GP filter
BVX1=[];
BVY1=[];
BVX2=[];
BVY2=[];
tic
for i=1:size(data,1)
    KernelMat=KernelFun(BVX,BVX,R,1)+diag(repmat([varNoise,0 0 0],1,size(BVX,1)));
    KernelTestTrain=KernelFun(data(i,1:3),BVX,R,1);
    KernelTest =KernelFun(data(i,1:3),data(i,1:3),R,1);
    % mean
    FunMean=[1,0,0,0]'+KernelTestTrain*(KernelMat\(BVY-repmat([1,0,0,0]',size(BVY,1)/4,1)));
    normal=(FunMean(2:end)'/norm(FunMean(2:end)));
    % variance
    %         KernelMat=KernelMat(1:4:end,1:4:end);
    %         KernelTestTrain=KernelTestTrain(1,1:4:end);
    %         KernelTest =KernelTest(1,1);
    tmp=[1 0 0 0];
    meanvar =tmp*KernelTest*tmp'-(tmp*KernelTestTrain)*(KernelMat\(KernelTestTrain'*tmp'));
    
    if meanvar>vthresh(1)
        BVX=[BVX;data(i,1:3)];
        BVY=[BVY;[0,data(i,4:6)]'];
        BVX1=[BVX1;data(i,1:3)];
        BVY1=[BVY1;[0,data(i,4:6)]'];
        disp('add one point from predicted value...');
    else if (normal*data(i,4:6)'<vthresh(2))
            if size(BVX2,1)<3
                BVX=[BVX;data(i,1:3)];
                BVY=[BVY;[0,data(i,4:6)]'];
                BVX2=[BVX2;data(i,1:3)];
                BVY2=[BVY2;[0,data(i,4:6)]'];
            else
                bvtemp=BVX2-repmat(data(i,1:3),size(BVX2,1),1);
                for j=1:size(bvtemp,1);
                    dist(j)=norm(bvtemp(j,:));
                end
                [dist,distid]=sort(dist);
                if dist(2)>0.1
                    BVX=[BVX;data(i,1:3)];
                    BVY=[BVY;[0,data(i,4:6)]'];
                    BVX2=[BVX2;data(i,1:3)];
                    BVY2=[BVY2;[0,data(i,4:6)]'];
                    disp('add one point from predicted normal...');
                end
            end
        end
    end
    %% fix the size of active data set;
    if size(BVX,1) > (nb+21)
        KernelMat=KernelFun(BVX,BVX,R,0)+varNoise*eye(size(BVX,1));
        KernelTestTrain=KernelFun(BVX,BVX,R,0);
        KernelTest =KernelFun(BVX,BVX,R,0);
        meanvar = KernelTest-KernelTestTrain*KernelMat\KernelTestTrain';
        meanvar = diag(meanvar);
        [~,inx]=sort(meanvar');
        BVX(inx(1),:)=[];
        BVY(inx(1)*4-3:inx(1)*4,:)=[];
    end
    disp(['step: ',num2str(i), ', NB of data point: ', num2str(size(BVX,1))]);
end
BVX_level=BVX;  %normalized data: all datapoints inside, on, and outside object surface for training
BVY_level=BVY;

BVX(:,1)=BVX(:,1)*inputscale+repmat(inputmean(1),size(BVX(:,1),1),1);
BVX(:,2)=BVX(:,2)*inputscale+repmat(inputmean(2),size(BVX(:,2),1),1);
BVX(:,3)=BVX(:,3)*inputscale+repmat(inputmean(3),size(BVX(:,3),1),1);

if(~isempty(BVX2))
    BVX2(:,1)=BVX2(:,1)*inputscale+repmat(inputmean(1),size(BVX2(:,1),1),1);
    BVX2(:,2)=BVX2(:,2)*inputscale+repmat(inputmean(2),size(BVX2(:,2),1),1);
    BVX2(:,3)=BVX2(:,3)*inputscale+repmat(inputmean(3),size(BVX2(:,3),1),1);
end

data_All_X=BVX(22:end,1:3);
data_All_Y=[BVY(86:4:end-2),BVY(87:4:end-1),BVY(88:4:end)];
data_output=[data_All_X,data_All_Y]; % filtered datapoints on the object surface
if (nargout>1&&~isempty(BVX2))
    data_curX=BVX2;  % datapoint from normal
    data_curY=[BVY2(2:4:end-2),BVY2(3:4:end-1),BVY2(4:4:end)];
    data_output_cur=[data_curX,data_curY];
end
toc
tic
if(bPlot)
    disp('------***** Plot the estimated surface *****-------')
    [xt,yt,zt]=meshgrid(-1.2:0.1:1.2,-1.2:0.1:1.2,-1:0.1:1);
    sizeTestData=size(xt);
    x_test=[xt(:),yt(:),zt(:)];
    KernelMat=KernelFun(BVX_level,BVX_level,R,1)+diag(repmat([varNoise,0 0 0],1,size(BVX_level,1)));
    FunValMean=zeros(size(x_test,1),1);
    alp = KernelMat\(BVY_level-repmat([1,0,0 0]',size(BVY_level,1)/4,1));
    InvK = inv(KernelMat(1:4:end,1:4:end));
   
    save alp;
    save BVX_level;
    save BVY_level;
    save inputmean;
    save inputscale;
    save R;
    %   pause;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    bModel = 1;  %% save model for AMPL
    if(bModel)
        
        delete('data_grasp.dat');
        A=1:size(BVX_level,1); %BVX_level: normalized data
        xtrain=[A',BVX_level];
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '%s', 'data;');
        fprintf(fid, '\n%s\n', 'param xtp: 1 2 3 :=');
        fclose(fid);
        dlmwrite('data_grasp.dat',xtrain,'delimiter', '\t','-append');
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '%s', ';');
        fclose(fid);
        
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '\n%s\n', 'param alp:=');
        fclose(fid);
        B=1:size(alp,1);
        alpnew=[B',alp];
        dlmwrite('data_grasp.dat',alpnew,'delimiter', '\t','-append');
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '%s', ';');
        fclose(fid);
        
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '\n%s\n', 'param RKernel:=');
        fprintf(fid,'%f\n',R);
        fprintf(fid, '%s', ';');
        fclose(fid);
        
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '\n%s\n', 'param inputscale:=');
        fprintf(fid,'%f\n',inputscale);
        fprintf(fid, '%s', ';');
        fclose(fid);
        
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '\n%s\n', 'param inputmean: 1 2 3 :=');
        fprintf(fid, '%d ',1);
        fprintf(fid,'%f %f %f',inputmean);
        fprintf(fid, '%s\n', ';');
        fclose(fid);  
        
        fid = fopen('data_grasp.dat', 'a');
        fprintf(fid, '\n%s', 'param InvK:');
        B=1:size(InvK,1);
        dlmwrite('data_grasp.dat',B,'delimiter', '\t','-append');
        fprintf(fid, '%s\n', ' :=');
        InvK=[B',InvK];
        dlmwrite('data_grasp.dat',InvK,'delimiter', '\t','-append');
        fprintf(fid, '%s\n', ';');
        fclose(fid);        
    end
    toc
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i=1:size(x_test,1)
        KernelTestTrain=KernelFun(x_test(i,1:3),BVX_level,R,1);
        %mean
        FunMean=[1,0,0,0]'+KernelTestTrain*(KernelMat\(BVY_level-repmat([1,0,0 0]',size(BVY_level,1)/4,1)));
        FunValMean(i)=FunMean(1);
        if(norm(x_test(i,1:3))>1.2) %
            FunValMean(i)=1;
        end
        %variance at each point
        tmp=[1 0 0,0];
        FunValVar(i) =tmp*KernelTest*tmp'-(tmp*KernelTestTrain)*(KernelMat\(KernelTestTrain'*tmp'));
    end
    
    xt=xt(:)*inputscale+repmat(inputmean(1),size(xt(:),1),1);
    yt=yt(:)*inputscale+repmat(inputmean(2),size(yt(:),1),1);
    zt=zt(:)*inputscale+repmat(inputmean(3),size(zt(:),1),1);
    xt=reshape(xt,sizeTestData);
    yt=reshape(yt,sizeTestData);
    zt=reshape(zt,sizeTestData);
    figure(2)
    qq=reshape(FunValMean,sizeTestData);
    FunValVar=FunValVar;
    Cdata= reshape(FunValVar,sizeTestData);
    psurf1=patch(isosurface(xt,yt,zt,qq,0.0001));
    hold on;
    isonormals(xt,yt,zt,qq,psurf1);
    isocolors(xt,yt,zt,Cdata,psurf1);
    set(psurf1,'FaceColor','interp','EdgeColor','none',...
        'FaceAlpha',0.8);hold on;
    camlight
    colorbar;
 	caxis([0,0.05]);
%   light('Position',[-1.0,-1.0,100.0],'Style','infinite');
    lighting phong;
    view(3);
    grid on;
    axis equal;
    
    %% plot the normal direction
    bNormal=1;
    if(bNormal)
    fv = isosurface(xt,yt,zt,qq,0.001);
    fvsample = fv.vertices(1:5:end,:);
    fvtest =(fvsample-repmat(inputmean,size(fvsample,1),1))/inputscale;
    for i = 1:size(fvtest,1)
    KernelTestTrain=KernelFun(fvtest(i,:),BVX_level,R,1);
    FunMean=[1,0,0,0]'+KernelTestTrain*(KernelMat\(BVY_level-repmat([1,0,0 0]',size(BVY_level,1)/4,1))); 
    fvtestnormal(i,:)=FunMean(2:4)'/norm(FunMean(2:4));
    end
    quiver3(fvsample(:,1),fvsample(:,2),fvsample(:,3),fvtestnormal(:,1),fvtestnormal(:,2),fvtestnormal(:,3),1.2,'color',[0.133333 0.545098 0.133333],...
        'LineWidth',1.5); hold on;
    end
    
   
end
end

