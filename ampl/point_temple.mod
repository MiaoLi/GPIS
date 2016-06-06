reset; 
option solver ipopt; 
 
option ipopt_options "max_iter=5000 tol=10e-8 acceptable_tol=10e-8 mu_strategy=adaptive bound_relax_factor=0.0001 dual_inf_tol=0.01 halt_on_ampl_error yes"; 
 
param pi := 3.14159; 

# number of fingers
param nb_doigts := 3; 

# the slack variable
param dv := 0.003; 
 
param nb_segments := 4 ; # nb of segments of each friction cone 
param mu := 0.8;         #coefficient de frottement 
 
# The fingertips positions in the object reference frame are given by pjk: 
var  p{j in 1..nb_doigts, k in 1..3}; 
var  pnew{j in 1..nb_doigts, k in 1..3};
var  pvar{j in 1..nb_doigts}; # the uncertainty at point p;
 
# The outward normal vector at a point p of the object is given by np (un-normalized):
#j: finger; k: vector component; 
var np{j in 1..nb_doigts, k in 1..3} ;

# The contact frame:
var vp{j in 1..nb_doigts, k in 1..3};
var vt1{j in 1..nb_doigts, k in 1..3};
var vt2{j in 1..nb_doigts, k in 1..3}; 
 
var phi{ i in 1..nb_segments, j in 1..nb_doigts} >= 0; 
var l{i in 1..nb_segments, j in 1..nb_doigts, k in 1..3}; 
var W{i in 1..nb_segments, j in 1..nb_doigts, k in 1..6}; 


#the training data points;
param datanb := 60;
param xtp{i in 1..datanb, j in 1..3};  
param alp{i in 1..datanb*4};
param RKernel;
param inputscale;
param inputmean{i in 1..3};
param InvK{i in 1..datanb, j in 1..datanb};
data data_grasp.dat;

# the objective function (quality + uncertainty): 
   minimize distance:  100000*(1/nb_doigts)*( sum{k in 1..3} ( sum{j in 1..3} p[j,k])^2  ); 
 
# fingertips on the object:
#subject to constraint_object_position{j in 1..3}: 1-dv <= ((p[j,1]/s1)^(2/epsilon2) + (p[j,2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (p[j,3]/s3)^(2/epsilon1) <= 1+dv ; 

#KerMat(1,1)=2*norm(x1-x2)^3-3*R*norm(x1-x2)^2+R^3;
#KerMat(1,2)=2*3*norm(x1-x2)*(-x1(1)+x2(1))-3*R*2*(-x1(1)+x2(1));
#KerMat(1,3)=2*3*norm(x1-x2)*(-x1(2)+x2(2))-3*R*2*(-x1(2)+x2(2));
#KerMat(1,4)=2*3*norm(x1-x2)*(-x1(3)+x2(3))-3*R*2*(-x1(3)+x2(3));
#norm(p1-xtp(i)) = sqrt(sum{k in 1..3}((p[j,k]-inputmean[k])/inputscale-xtp[i,k])^2)

subject to constraint_data_normalization{j in 1..nb_doigts, k in 1..3}: pnew[j,k] = (p[j,k]-inputmean[k])/inputscale;

subject to constraint_point_uncertainty_1{j in 1..nb_doigts}:pvar[j] <=0.03;
# compute variance at each point
subject to constraint_point_uncertainty{j in 1..nb_doigts}: pvar[j] = RKernel^3 - sum{n in 1..datanb}(sum{m in 1..datanb}(InvK[m,n]*(2*(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[m,k])^2))^3-3*RKernel*(sum{k in 1..3}(pnew[j,k]-xtp[m,k])^2)+RKernel^3)*(2*(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[n,k])^2))^3-3*RKernel*(sum{k in 1..3}(pnew[j,k]-xtp[n,k])^2)+RKernel^3)));

subject to constraint_object_position{j in 1..3}: 0.00001 <= 1+sum{i in 1..datanb}(alp[4*i-3]*(2*(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))^3-3*RKernel*(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2)+RKernel^3) + alp[4*i-2]*(2*3*sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2)*(-pnew[j,1]+xtp[i,1]) - 3*RKernel*2*(-pnew[j,1]+xtp[i,1])) + alp[4*i-1]*(2*3*sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2)*(-pnew[j,2]+xtp[i,2]) - 3*RKernel*2*(-pnew[j,2]+xtp[i,2])) + alp[4*i]*(2*3*sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2)*(-pnew[j,3]+xtp[i,3]) - 3*RKernel*2*(-pnew[j,3]+xtp[i,3]))) <= 0.0002;

#KerMat(2,1)=2*3*norm(x1-x2)*(x1(1)-x2(1))-3*R*2*(x1(1)-x2(1));
#KerMat(2,2)=2*3*(-x1(1)+x2(1))/norm(x1-x2)*(x1(1)-x2(1))+2*3*norm(x1-x2)*(-1)-3*R*2*(-1);
#KerMat(2,3)=2*3*(-x1(2)+x2(2))/norm(x1-x2)*(x1(1)-x2(1));
#KerMat(2,4)=2*3*(-x1(3)+x2(3))/norm(x1-x2)*(x1(1)-x2(1));

# the normal direction:
subject to contraint_np_1{j in 1..3}: np[j,1] = sum{i in 1..datanb}(alp[4*i-3]*(2*3*sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2)*(pnew[j,1]-xtp[i,1])-3*RKernel*2*((pnew[j,1]-xtp[i,1]))) + alp[4*i-2]*(2*3*((-pnew[j,1]+xtp[i,1]))/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,1]-xtp[i,1])+2*3*(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(-1)-3*RKernel*2*(-1)) + alp[4*i-1]*(2*3*(-pnew[j,2]+xtp[i,2])/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,1]-xtp[i,1])) + alp[4*i]*(2*3*(-pnew[j,3]+xtp[i,3])/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,1]-xtp[i,1]))); 

 #KerMat(3,1)=2*3*norm(x1-x2)*(x1(2)-x2(2))-3*R*2*(x1(2)-x2(2));
 #KerMat(3,2)=2*3*(-x1(1)+x2(1))/norm(x1-x2)*(x1(2)-x2(2));
 #KerMat(3,3)=2*3*(-x1(2)+x2(2))/norm(x1-x2)*(x1(2)-x2(2))+2*3*norm(x1-x2)*(-1)-3*R*2*(-1);
 #KerMat(3,4)=2*3*(-x1(3)+x2(3))/norm(x1-x2)*(x1(2)-x2(2));

subject to contraint_np_2{j in 1..3}: np[j,2] = sum{i in 1..datanb}(alp[4*i-3]*(2*3*sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2)*(pnew[j,2]-xtp[i,2])-3*RKernel*2*((pnew[j,2]-xtp[i,2]))) + alp[4*i-2]*(2*3*(-pnew[j,1]+xtp[i,1])/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,2]-xtp[i,2])) + alp[4*i-1]*(2*3*((-pnew[j,2]+xtp[i,2]))/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,2]-xtp[i,2])+2*3*(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(-1)-3*RKernel*2*(-1)) + alp[4*i]*(2*3*(-pnew[j,3]+xtp[i,3])/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,2]-xtp[i,2])));

#KerMat(4,1)=2*3*norm(x1-x2)*(x1(3)-x2(3))-3*R*2*(x1(3)-x2(3));
#KerMat(4,2)=2*3*(-x1(1)+x2(1))/norm(x1-x2)*(x1(3)-x2(3));
#KerMat(4,3)=2*3*(-x1(2)+x2(2))/norm(x1-x2)*(x1(3)-x2(3));
#KerMat(4,4)=2*3*(-x1(3)+x2(3))/norm(x1-x2)*(x1(3)-x2(3))+2*3*norm(x1-x2)*(-1)-3*R*2*(-1);

subject to contraint_np_3{j in 1..3}: np[j,3] = sum{i in 1..datanb}(alp[4*i-3]*(2*3*sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2)*(pnew[j,3]-xtp[i,3])-3*RKernel*2*((pnew[j,3]-xtp[i,3]))) + alp[4*i-2]*(2*3*(-pnew[j,1]+xtp[i,1])/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,3]-xtp[i,3])) + alp[4*i-1]*(2*3*(-pnew[j,2]+xtp[i,2])/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,3]-xtp[i,3]))+ alp[4*i]*(2*3*((-pnew[j,3]+xtp[i,3]))/(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(pnew[j,3]-xtp[i,3])+2*3*(sqrt(sum{k in 1..3}(pnew[j,k]-xtp[i,k])^2))*(-1)-3*RKernel*2*(-1)));



#constraints for the contact frame:

subject to constraint_vp_norm{j in 1..3}: 1-dv<=vp[j,1]^2+vp[j,2]^2+vp[j,3]^2<=1+dv;
subject to constraint_vp_direction{j in 1..3}: vp[j,1]*np[j,1]+vp[j,2]*np[j,2]+vp[j,3]*np[j,3]>=0;
subject to constraint_vp_1{j in 1..3}: 0-dv<= vp[j,2]*np[j,3]-vp[j,3]*np[j,2]<=0+dv;
subject to constraint_vp_2{j in 1..3}: 0-dv<= vp[j,3]*np[j,1]-vp[j,1]*np[j,3]<=0+dv;
subject to constraint_vp_3{j in 1..3}: 0-dv<= vp[j,1]*np[j,2]-vp[j,2]*np[j,1]<=0+dv;


subject to constraint_vt1_1{j in 1..3}: 1-dv<=vt1[j,1]^2+vt1[j,2]^2+vt1[j,3]^2<=1+dv;
subject to constraint_vt1_2{j in 1..3}: 0-dv<=vt1[j,1]*vp[j,1]+vt1[j,2]*vp[j,2]+vt1[j,3]*vp[j,3]<=0+dv;

subject to constraint_vt2{j in 1..3}: 1-dv<=vt2[j,1]^2+vt2[j,2]^2+vt2[j,3]^2<=1+dv;
subject to constraint_vt2_2{j in 1..3}: 0-dv<=vt2[j,1]*vp[j,1]+vt2[j,2]*vp[j,2]+vt2[j,3]*vp[j,3]<=0+dv;

subject to constraint_cross_product_1{j in 1..3}: 0-dv<=vt1[j,2]*vt2[j,3]-vt1[j,3]*vt2[j,2]-vp[j,1]<=0+dv;
subject to constraint_cross_product_2{j in 1..3}: 0-dv<=vt1[j,3]*vt2[j,1]-vt1[j,1]*vt2[j,3]-vp[j,2]<=0+dv;
subject to constraint_cross_product_3{j in 1..3}: 0-dv<=vt1[j,1]*vt2[j,2]-vt1[j,2]*vt2[j,1]-vp[j,3]<=0+dv;

# the contact primitive:
# l[i,j,k] i frition segment, j finger, k vector components. 
subject to constraint_contact_primitive_1{i in 1..nb_segments, j in 1..3, k in 1..3}: l[i,j,k] = vt1[j,k]*mu*cos(6.28*i/nb_segments)+vt2[j,k]*mu*sin(6.28*i/nb_segments)+vp[j,k];

# the contact primitive wrench:

subject to contraint_W_12_1{i in 1..nb_segments, j in 1..3, k in 1..3}: W[i,j,k] = l[i,j,k]; 
subject to contraint_W_12_4{i in 1..nb_segments, j in 1..3}: W[i,j,4] = l[i,j,3]*p[j,2]-l[i,j,2]*p[j,3]; 
subject to contraint_W_12_5{i in 1..nb_segments, j in 1..3}: W[i,j,5] = -l[i,j,3]*p[j,1]+l[i,j,1]*p[j,3]; 
subject to contraint_W_12_6{i in 1..nb_segments, j in 1..3}: W[i,j,6] = l[i,j,2]*p[j,1]-l[i,j,1]*p[j,2]; 

# Force closure constraint 

subject to constraint_fc1:   1-dv <= sum{i in 1..nb_segments} sum{j in 1..nb_doigts} phi[i,j]  <= 1+dv ; 
subject to constraint_fc2{k in 1..6}:   0-dv <= sum{i in 1..nb_segments} sum{j in 1..nb_doigts} phi[i,j]*W[i,j,k]  <= 0+dv ; 

#force closure constraints end 
 
 
#initialisation 
 
let {i in 1..nb_segments, j in 1..nb_doigts} phi[i,j] := 0.0417;
 
let p[1,1] := 
let p[1,2] := 
let p[1,3] := 
let p[2,1] := 
let p[2,2] := 
let p[2,3] := 
let p[3,1] := 
let p[3,2] := 
let p[3,3] := 

let pnew[1,1] := 0.1; 
let pnew[1,2] := 0.1;
let pnew[1,3] := 0.1;

let pnew[2,1] := 0.2;
let pnew[2,2] := 0.2;
let pnew[2,3] := 0.2;

let pnew[3,1] := 0.3;
let pnew[3,2] := 0.3;
let pnew[3,3] := 0.3;

let {j in 1..3} vp[j,1] :=0;
let {j in 1..3} vp[j,2] :=0;
let {j in 1..3} vp[j,3] :=1;

let {j in 1..3} vt1[j,1] :=0.7071;
let {j in 1..3} vt1[j,2] :=-0.7071;
let {j in 1..3} vt1[j,3] :=0;

let {j in 1..3} vt2[j,1] :=0.7071;
let {j in 1..3} vt2[j,2] :=0.7071;
let {j in 1..3} vt2[j,3] :=0; 

#Solve the problem  
solve;  
  
# Print the solution 
display solve_message
print 'Solve time:' 
display _solve_time
print 'Objective:'
display  p  
display phi 
display l 
display np 
display vp
