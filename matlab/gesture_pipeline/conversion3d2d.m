% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [ xp ] = conversion3d2d( XYZ )
%This function transfers 3D image to 2D image using calibration matrix of
%the kinect camera and Camera Calibration toollbox
%   Detailed explanation goes here

% 3D2IM
%clear all;
%close all;
%leftXYZ = textread('left.txt', '', 'delimiter', ',','emptyvalue', NaN);
%XYZ = leftXYZ(:,[4:6]);

%%XYZ_changed = [XYZ(:,2)';XYZ(:,3)';XYZ(:,1)'];
XYZ_changed = XYZ';
% % XYZ_Q = leftXYZ(:,[7:10]);
% figure(1);
% plot3(XYZ_changed(1,:),XYZ_changed(2,:),XYZ_changed(3,:),'r.');
% hold on
% plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'b.');
% grid on
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

Camera_Calibration_Matrix_Depth = [ 587.04607160    0       317.39001517    0;
                                    0               587.04  234.30080720    0;
                                    0               0       1               0 ];
                                
Datapoints_Translation = XYZ_changed;


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
hold on
plot(xp(1,:),xp(2,:),'r.');
end

