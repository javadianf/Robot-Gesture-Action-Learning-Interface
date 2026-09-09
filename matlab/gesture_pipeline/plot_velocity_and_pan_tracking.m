% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

clear all;
close all;
reply = input('Please enter the file name(TXT): ', 's');
% data_mat = textread(reply,'%f','headerlines',9);
DELIMITER = ' ';
HEADERLINES = 9;

% Import the file
data_mat_ = importdata(reply, DELIMITER, HEADERLINES);
% Create new variables in the base workspace from those fields.
vars = fieldnames(data_mat_);
for i = 1:length(vars)
    assignin('base', vars{i}, data_mat_.(vars{i}));
end
data_mat = data_mat_.data;
time_bias=data_mat(1,size(data_mat,2));
data_mat(:,size(data_mat,2))=data_mat(:,size(data_mat,2))-time_bias;
rotational_vel_set = data_mat(:,2)*(180/pi);
rotational_vel_robot = data_mat(:,4)*(180/pi);
translational_vel_set = data_mat(:,1);
translational_vel_robot = data_mat(:,3);
left_error = data_mat(:,5);
right_error = data_mat(:,6);
average_error = data_mat(:,7);
pan_angle = data_mat(:,8);
rotation_real = data_mat(:,9);
rotation_normalized = data_mat(:,10);
distance_m = data_mat(:,11);
local_pose_x = data_mat(:,12);
local_pose_y = data_mat(:,13);
local_quaternion_z = data_mat(:,14);
local_quaternion_w = data_mat(:,15);
robot_pose_x = data_mat(:,16);
robot_pose_y = data_mat(:,17);
robot_quaternion_z = data_mat(:,18);
robot_quaternion_w = data_mat(:,19);
time_elapsed = data_mat(:,20)/1000000;
%quatrenion to angle save
%plotting figure 1
% figure(1)
% plot(data_mat(:,size(data_mat,2)),data_mat(:,1),data_mat(:,size(data_mat,2)),data_mat(:,3),'r');
% title('translational velocity received by the robot vs translational velocity set to the robot')
% %plotting figure 2
% figure(2)
% plot(data_mat(:,size(data_mat,2)),data_mat(:,2),data_mat(:,size(data_mat,2)),data_mat(:,4),'r');
% title('rotational velocity received by the robot vs rotational velocity set to the robot')
% %plotting figure 3
% figure(3)
% plot(data_mat(:,size(data_mat,2)),data_mat(:,5),'m',data_mat(:,size(data_mat,2)),-data_mat(:,6),'c',data_mat(:,size(data_mat,2)),data_mat(:,7),'g',data_mat(:,size(data_mat,2)),data_mat(:,8),'r');
% title('left error:magneta  right errro:cyan  ave:green  pan angle:red')
% %plotting figure 4
% figure(4)
% subplot(2,2,1)
% plot(data_mat(:,size(data_mat,2)),data_mat(:,2));
% %plotting figure 5
% figure(5)
figure(1)
subplot(3,2,1)
plot(time_elapsed,data_mat(:,1),time_elapsed,data_mat(:,3),'r');
title('translational vel of robot(red) vs translational vel set(m per s)')
subplot(3,2,3)
plot(time_elapsed,data_mat(:,2),time_elapsed,data_mat(:,4),'r');
title('rotational vel received of robot(RED) vs rotational vel set (radian per sec) ')
subplot(3,2,5)
plot(time_elapsed,data_mat(:,5),'m',time_elapsed,-data_mat(:,6),'c',time_elapsed,data_mat(:,7),'g');
title('left_error:magneta right_errro:cyan ave:green (pixel)' )
subplot(3,2,2)
% plot(time_elapsed,data_mat(:,1),time_elapsed,data_mat(:,11),'r');
% title('translational vel set to the robot vs point_line_distance(m)')
plot(time_elapsed,data_mat(:,11),'c');
title('distance of the user from the robot (m)')
subplot(3,2,4)
plot(time_elapsed,-data_mat(:,10),time_elapsed,-data_mat(:,9),'r');
title('real rotational steering_angle(red) vs normalized rotational steering_angle(degree)');
subplot(3,2,6)
plot(time_elapsed,data_mat(:,8));
title('pan steering_angle(degree)' )
% plot(time_elapsed,data_mat(:,10),time_elapsed,data_mat(:,2),'r');
% title('normalized rotational steering_angle(degree) vs rotational vel(degree per s)')
figure(2)
% subplot(5,1,1)
% plot(data_mat(:,13),data_mat(:,12),'r');
% title('XY trajectory from localization')
% subplot(5,1,3)
% plot(data_mat(:,17),data_mat(:,16),'m');
% title('XY trajectory from the robot')
subplot(2,1,1)
% % plot(heading_local,heading_robot);
% % title('heading trajectory from the robot vs localozation')
xy_local = data_mat(:,12:13);
plot_trajectory_map(xy_local);
title('XY trajectory in the map')
v = [0,0];
flag = 0;
subplot(2,1,2)
for n=1:size(data_mat,1)
    if(abs(data_mat(n,12))>0 && flag==0)
        v(1,1)=data_mat(n,12);
        v(1,2)=data_mat(n,13);
        flag = 1;
    end
end
 xy_robot = data_mat(:,16:17);
for  m=1:size(data_mat,1) 
    xy_robot(m,:) = xy_robot(m,:) + v;
end
plot_trajectory_map(xy_robot);
title('XY trajectory from the robot in the map')
% subplot(2,1,1)
% plot(xy_robot(:,1),xy_robot(:,2),'m');
% title('XY trajectory from the robot rectified')

% figure(3)
% subplot(2,1,1)
% q=zeros(size(data_mat,1),4);
% q(:,1)=data_mat(:,14); % omega
% q(:,4)=data_mat(:,15); % z
% R=zeros(size(data_mat,1),4);
% [R(:,1) R(:,2) R(:,3)] = quat2angle(q);
% R(:,4)=R(:,1)*(180/pi);
% plot(time_elapsed,R(:,4));
% subplot(2,1,2)
% q_=zeros(size(data_mat,1),4);
% q_(:,1)=data_mat(:,18); % omega
% q_(:,4)=data_mat(:,19); % z
% R_=zeros(size(data_mat,1),4);
% [R_(:,1) R_(:,2) R_(:,3)] = quat2angle(q_);
% R_(:,4)=R_(:,1)*(180/pi);
% plot(time_elapsed,R_(:,4));