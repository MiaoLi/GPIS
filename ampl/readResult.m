function   [solve_time,points] = readResult(fname)

%  Hand_pos: 9x1 vector the cos and sin of 3 Euler angles and the postion
%  Finger_joint: the cos and sin of finger joint;
%  open file for input, include error handling
fid = fopen(fname,'r');
if fid < 0
   error(['Could not open ',fname,' for input']);
end

for i=1:4
    buffer = fgetl(fid);
end

tline = fgetl(fid);
 [a,b,c] = strread(tline,'%s %s %f');
solve_time = c;

for i=1:3
    buffer = fgetl(fid);
end

for i=1:3
    for j=1:3
tline = fgetl(fid);
[a,b,c] = strread(tline,'%f %f %f');
p(i,j) = c;
    end
end
points=p;


