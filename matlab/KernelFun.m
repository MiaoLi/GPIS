%%
%KernelMat = KernelFun(x,y,mu,lamda)
%x,y: each row is an observation point, 3 dimendion
%mu,lamda: kernel parameters.

function KernelMat = KernelFun(x,y,R,dev)
xdatasize=size(x,1);
ydatasize=size(y,1);
KernelCell =cell(xdatasize,ydatasize);

for i=1:xdatasize
    for j=1:ydatasize
        switch dev
            case 0
                KernelCell{i,j}=KernelBlock(x(i,:),y(j,:),R);
            case 1
                KernelCell{i,j}=KernelBlock1(x(i,:),y(j,:),R);
        end
    end
end
KernelMat = cell2mat(KernelCell);
end

%%
%computer the kernel blocks
function KerMat = KernelBlock(x1,x2,R)
KerMat=zeros(1,1);
KerMat(1,1)=2*norm(x1-x2)^3-3*R*norm(x1-x2)^2+R^3;
% KerMat(1,1)=-norm(x1-x2)^2*log(norm(x1-x2)+realmin)+R*0;
end
%%
function KerMat = KernelBlock1(x1,x2,R)
if norm(x1-x2)>1e-4
    KerMat=zeros(4,4);
    KerMat(1,1)=2*norm(x1-x2)^3-3*R*norm(x1-x2)^2+R^3;
    KerMat(1,2)=2*3*norm(x1-x2)*(-x1(1)+x2(1))-3*R*2*(-x1(1)+x2(1));
    KerMat(1,3)=2*3*norm(x1-x2)*(-x1(2)+x2(2))-3*R*2*(-x1(2)+x2(2));
    KerMat(1,4)=2*3*norm(x1-x2)*(-x1(3)+x2(3))-3*R*2*(-x1(3)+x2(3));
    
    KerMat(2,1)=2*3*norm(x1-x2)*(x1(1)-x2(1))-3*R*2*(x1(1)-x2(1));
    KerMat(2,2)=2*3*(-x1(1)+x2(1))/norm(x1-x2)*(x1(1)-x2(1))+2*3*norm(x1-x2)*(-1)-3*R*2*(-1);
    KerMat(2,3)=2*3*(-x1(2)+x2(2))/norm(x1-x2)*(x1(1)-x2(1));
    KerMat(2,4)=2*3*(-x1(3)+x2(3))/norm(x1-x2)*(x1(1)-x2(1));
    
    KerMat(3,1)=2*3*norm(x1-x2)*(x1(2)-x2(2))-3*R*2*(x1(2)-x2(2));
    KerMat(3,2)=2*3*(-x1(1)+x2(1))/norm(x1-x2)*(x1(2)-x2(2));
    KerMat(3,3)=2*3*(-x1(2)+x2(2))/norm(x1-x2)*(x1(2)-x2(2))+2*3*norm(x1-x2)*(-1)-3*R*2*(-1);
    KerMat(3,4)=2*3*(-x1(3)+x2(3))/norm(x1-x2)*(x1(2)-x2(2));
    
    KerMat(4,1)=2*3*norm(x1-x2)*(x1(3)-x2(3))-3*R*2*(x1(3)-x2(3));
    KerMat(4,2)=2*3*(-x1(1)+x2(1))/norm(x1-x2)*(x1(3)-x2(3));
    KerMat(4,3)=2*3*(-x1(2)+x2(2))/norm(x1-x2)*(x1(3)-x2(3));
    KerMat(4,4)=2*3*(-x1(3)+x2(3))/norm(x1-x2)*(x1(3)-x2(3))+2*3*norm(x1-x2)*(-1)-3*R*2*(-1);
else
    KerMat=zeros(4,4);
    KerMat(1,1)=2*norm(x1-x2)^3-3*R*norm(x1-x2)^2+R^3;
    KerMat(2,2)=6*R;
    KerMat(3,3)=6*R;
    KerMat(4,4)=6*R;
    
end
end
%%