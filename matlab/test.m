function test()
load alp.mat;
load inputmean;
load inputscale;
load BVX_level;
load BVY_level;
load R;
atmp=[1 1   -0.00708876
1 2   -0.0118505
1 3    0.00979608
2 1   -0.0246533
2 2    0.00683552
2 3    0.15903
3 1   -0.00522296
3 2    0.029688
3 3    0.0626653];
a=atmp(4:6,3)';
% a=(a-inputmean)/inputscale;
KernelTestTrain=KernelFun(a,BVX_level,R,1);
KernelTestTrain*alp+[1,0,0,0]'
FunMean=[1,0,0,0]'+KernelTestTrain*(KernelMat\(BVY_level-repmat([1,0,0 0]',size(BVY_level,1)/4,1)))


end

