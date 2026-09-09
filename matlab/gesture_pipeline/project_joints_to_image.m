% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

%% 3D2IM
clear all;
close all;
leftXYZ = textread('left.txt', '', 'delimiter', ',','emptyvalue', NaN);
XYZ = leftXYZ(:,[4:6]);
XYZ_changed = [XYZ(:,2)';XYZ(:,3)';XYZ(:,1)'];
% XYZ_Q = leftXYZ(:,[7:10]);
figure(1);
plot3(XYZ_changed(1,:),XYZ_changed(2,:),XYZ_changed(3,:),'r.');
hold on
plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'b.');
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');

Camera_Calibration_Matrix_Depth = [ 587.04607160    0       317.39001517    0;
                                    0               587.04  234.30080720    0;
                                    0               0       1               0 ];
                                
Datapoints_Translation = XYZ_changed;
% Datapoints_Rotation_Quartenion = XYZQ;

% Rotation expressed as Quartenions
% Q0 = Datapoints_Rotation_Quartenion(:,1);
% Q1 = Datapoints_Rotation_Quartenion(:,2);
% Q2 = Datapoints_Rotation_Quartenion(:,3);
% Q3 = Datapoints_Rotation_Quartenion(:,4);
% 
% % Rotation matrix elements
% R11 = Q0.^2+Q1.^2+Q2.^2+Q3.^2;
% R12 = 2*((Q1.*Q2)-(Q0.*Q3));
% R13 = 2*((Q0.*Q2)+(Q1.*Q3));
% R21 = 2*((Q1.*Q2)+(Q0.*Q3));    
% R22 = Q0.^2-Q1.^2+Q2.^2-Q3.^2;
% R23 = 2*((Q2.*Q3)-(Q0.*Q1));    
% R31 = 2*((Q1.*Q3)-(Q0.*Q2));    
% R32 = 2*((Q0.*Q1)+(Q2.*Q3));    
% R33 = Q0.^2-Q1.^2-Q2.^2+Q3.^2;
% 
% % Get the rotation vector
% Rotmatrix = [R11 R12 R13;
%              R21 R22 R23;
%              R31 R32 R33];
% 
% om = rodrigues(Rotmatrix);

OM = [1 0 0;
      0 1 0;
      0 0 1];
om_theta = rodrigues(OM);
T = [ 0;0;0];
f = [587.04607160;587.04607160];
c = [317.39;234.3];
k = [ 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000 ];
alpha = 0;
[xp,dxpdX,dxpdom,dxpdT,dxpdf,dxpdc,dxpdk,dxpdalpha] = project_points3(Datapoints_Translation,om_theta,T,f,c,k,alpha);

figure(2);
plot(xp(1,:),xp(2,:),'r.');