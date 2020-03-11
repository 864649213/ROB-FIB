%%% First we clean the workspace

clc;
clear;

%%% We define the initial and final poses and the transformation to be
%%% interpolated

TAn = transl(0.1, 0.25, -0.5)*trotx(pi)*troty(pi);
TBn = transl(0.25, 0.1, 0.5)*trotx(pi/2)*troty(pi/2)*trotz(pi/2);
TABn = inv(TAn)*TBn;

%%% Orientation to be interpolated

Rn = t2r(TABn);

%%% Angle-vector representation

[alpha_n vector_n] = tr2angvec(Rn);

%%% Euler angles

angles_n = tr2eul(Rn);

%%% Interpolation based on the angle-vector representation

n=200;

for i=1:n+1
   TRAJ1_n(:,:,i) = TAn*transl((i-1)/n*transl(TABn))*...
       angvec2tr(alpha_n*(i-1)/n, vector_n);
end

%%% Interpolation based on Euler angles

for i=1:n+1
   TRAJ2_n(:,:,i) = TAn*transl((i-1)/n*transl(TABn))*...
       eul2tr(angles_n(1)*(i-1)/n, angles_n(2)*(i-1)/n, angles_n(3)*(i-1)/n);
end

%%% Interpolation using quaternion spherical interpolation

TRAJ3_n = ctraj(TAn, TBn, (0:1/n:1));