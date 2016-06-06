function plot_obj( filename)
 [V,F] = read_vertices_and_faces_from_obj_file(filename);
 V=[1 0 0;0 0 1; 0 -1 0]'*V';  % change the y and z axis
 V=V';
 
 % plot3(V(:,1),V(:,2),V(:,3),'r.'); hold on;
 
 trisurf(F,V(:,1),V(:,2),V(:,3),'FaceColor',[0.839216 0.839216 0.839216],'EdgeColor','none','FaceAlpha',0.6),hold on;
 title('Object Point Cloud','FontSize',18);
 light('Position',[-1.0,-1.0,100.0],'Style','infinite');
%  daspect([1,1,1]);
 view(3);
%  camlight 
%  lighting phong
% %  size(V)
axis equal
end
