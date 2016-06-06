function dd = convert_vf_to_data( V, F )
%CONVERT_VF_TO_DATA Summary of this function goes here
%   Detailed explanation goes here

dd=[];
N = size(V,2);
M = size(V,1);
dd = [V';zeros(N,M)];
%%
for i=1:size(F,1)
    
    fn = cross(V(F(i,1),:)'-V(F(i,2),:)', V(F(i,1),:)'-V(F(i,3),:)' );
    
    for j=1:3
        dd(N+1:end,F(i,j)) = dd(N+1:end,F(i,j)) + fn;
    end
    
end

for i=1:size(dd,2)
   dd(N+1:end,i) = dd(N+1:end,i)/norm(dd(N+1:end,i));
end