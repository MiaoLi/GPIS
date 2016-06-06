function plot_sphere(pcenter,r,nb,color)

%% plot a sphere at given point
%% pcenter: N x 3, the center of the sphere
%% r: radius of sphere
%% nb: number of faces
   if nargin<4
   color =[65 29 226]/255;
   end
   
   [x,y,z] = sphere(nb);
   datasize=size(x);
   data =r*[x(:),y(:),z(:)];

   for i=1:size(pcenter,1)
   datanew = data + repmat([pcenter(i,1),pcenter(i,2),pcenter(i,3)],size(data,1),1);    
   x=reshape(datanew(:,1),datasize);
   y=reshape(datanew(:,2),datasize);
   z=reshape(datanew(:,3),datasize);
   surf(x,y,z,'FaceColor',color,'EdgeColor','none','FaceAlpha',1);hold on;   
   end
   
end

