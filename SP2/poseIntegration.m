function [Pose_t,Pose_est,Pk] = poseIntegration(V,Pk0,IC,L,R,S) % IC->Initial Condition [xo,yo,th0]
                                   % R and L are the displacements of
                                   % wheels [m]
persistent x_w y_w suma_theta  %  internal variable to be mantained its values
persistent Pose_est1 Pk1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%   Read Initial condition   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_ini=IC(1);
y_ini=IC(2);
theta_ini=IC(3);
P_ini=Pk0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%   variable initialitation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isempty (x_w)
    x_w=x_ini;
    y_w=y_ini;
    suma_theta=theta_ini; 
    Pose_est1 = [x_w;y_w;suma_theta];
    Pk1 = P_ini;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%  compute the odometry    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_th = (R-L)/(2*S);
delta_d = (R+L)/2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%  integration of the robot pose %%%%%%%%%%%%%%%%%%%%%%%%%

x_w = x_w + delta_d * cos(suma_theta);
y_w = y_w + delta_d * sin(suma_theta);
suma_theta = mod((suma_theta + delta_th + V(2,2)) ,2*pi);

Pose_t = [x_w;y_w;suma_theta];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%  Build the Jacocian %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

F_x = [[1 0 -delta_d*sin(suma_theta+delta_th)];....
      [0 1 delta_d*cos(suma_theta+delta_th)];....
      [0 0 1]];

F_v = [[cos(suma_theta+delta_th) -delta_d*sin(suma_theta+delta_th)];....
    [sin(suma_theta+delta_th) delta_d*cos(suma_theta+delta_th)];...
    [0 1]];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% Compute the EKF Trucated Taylor %%%%%%%%%%%%%%%%%%%%%%%%

Pose_est1 = Pose_est1 + F_x*(Pose_t - Pose_est1) + F_v*diag(V);

Pk1 = F_x*Pk1*F_x' + F_v*V*F_v';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%  output variable %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Pose_est = Pose_est1;
Pk = Pk1;

end