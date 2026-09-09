% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

%This program need the following functions:
%angle, joint_parse_test , conversion3d2d

clear all;
close all;
reply = input('Please enter the file name(TXT): ', 's');
joint_positions = joint_parse_alt_log(reply);
head_position = (conversion3d2d( joint_positions(:,:,1 )))'
torso_position = (conversion3d2d( joint_positions(:,:,4 )))'
left_hand_position = (conversion3d2d( joint_positions(:,:,2 )))';
right_hand_position = (conversion3d2d( joint_positions(:,:,3 )))';
neck_position = (conversion3d2d( joint_positions(:,:,5 )))'


frame_total=size(joint_positions,1);
for i=1:frame_total
    [ angle_lines(i) , angle_right(i) , angle_left(i), mix_angle(i)] = steering_angle( head_position(i,:), torso_position(i,:), neck_position(i,:), right_hand_position(i,:) , left_hand_position(i,:));
end



figure(3)
hold on
plot(angle_lines,'c');
angle_lines = medfilt1(angle_lines,40);
plot(angle_lines);
K_rel = 1;
for i=1:(frame_total-1)
velocity(i) = K_rel * (angle_lines(i+1)-angle_lines(i));
end

figure(4)
hold on
plot(velocity);

%Filter the noise to have a smooth movement
y = medfilt1(velocity,14);
% D = fdesign.lowpass('Fp,Fst,Ap,Ast',0.1,0.15,1,20);
% Hd = design(D,'equiripple','StopBandShape','linear','StopBandDecay',20);
% %Hd = design(D);
% y = filter(Hd,velocity);
 plot(y,'m');
%one way of filtering
%plot(velocity.*abs(velocity)/max(abs(velocity)),'r-')
%filtering
%[B,A]=butter(2,0.01,'low');
% Y=filter(B,A,velocity);


%% test
%[t,r,l]=steering_angle([0,10],[0,-10],[0,0],[10,-10],[-10,10])
%[t,r,l]=steering_angle([0,10],[0,-10],[0,0],[-10,0],[10,0])