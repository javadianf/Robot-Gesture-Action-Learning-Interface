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
data_mat=data_mat(1:size(data_mat,1)-1,:);
i=2;
while data_mat(i,1) == 0 && data_mat(i,2) == 0
    i=i+1;
end
time_bias=data_mat(i,20);
data_mat=data_mat(i:size(data_mat,1),:);
data_mat(:,20)=data_mat(:,20)-time_bias;
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
time_elapsed = data_mat(:,20)/100000;

figure(1)
subplot(2,1,1)
xy_local = data_mat(:,12:13);
plot_trajectory_map(xy_local);
title('XY trajectory from localization in the map','FontSize',18)
xlabel('X','FontSize',16)
ylabel('Y','FontSize',16)
v = [0,0];
flag = 0;
set(gca,'FontSize',14)
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
xlabel('X','FontSize',16)
ylabel('Y','FontSize',16)
set(gca,'FontSize',14)
title('XY trajectory from the robot odometry in the map','FontSize',18)

figure(2)

q=zeros(size(data_mat,1),4);
q(:,1)=data_mat(:,14); % omega
q(:,4)=-10*data_mat(:,15); % z
R=zeros(size(data_mat,1),4);
[R(:,1) R(:,2) R(:,3)] = quat2angle(q);
R(:,4)=R(:,1)*(180/pi);
i=1;
while isnan(R(i,4))
    i=i+1;
end
R_temp = R(i:size(R,1),:);

plot(time_elapsed,R(:,4)*pi/180,'k','LineWidth',2);
title('Orientation trajectory of the robot','FontSize',18)
xlabel('Time [sec]','FontSize',16)
ylabel('Robot orientation [rad]','FontSize',16)
set(gca,'FontSize',14)

figure(3)
plot(time_elapsed,data_mat(:,1),'k','LineWidth',3)
hold on
plot(time_elapsed,data_mat(:,3),'k--','LineWidth',1);
title('Translational velocity','FontSize',18)
xlabel('Time [sec]','FontSize',16)
ylabel('Translational velocity [m/s]','FontSize',16)
legend('Set value','Value from robot')
set(gca,'FontSize',14)


figure(4)
plot(time_elapsed,data_mat(:,2),'k','LineWidth',2)
hold on
plot(time_elapsed,data_mat(:,4),'k--','LineWidth',1);
title('Rotational velocity','FontSize',18)
xlabel('Time [sec]','FontSize',16)
ylabel('Rotational velocity [rad/s]','FontSize',16)
legend('Set value','Value from robot')
set(gca,'FontSize',14)

figure(5)
Curveture_set = rotational_vel_set ./ translational_vel_set;
for i=1:size(Curveture_set,1)
    if Curveture_set(i) >= 5000 || Curveture_set(i) <= -5000
        Curveture_set(i)=0;
    end
end
Curveture_robot = rotational_vel_robot ./ translational_vel_robot;
for i=1:size(Curveture_robot,1)
    if Curveture_robot(i) >= 5000 || Curveture_robot(i) <= -5000
        Curveture_robot(i)=0;
    end
end
plot(time_elapsed,Curveture_set,'k','LineWidth',2);
title('Curveture trajectory','FontSize',18)
%legend('Set cerveture','Robot Curveture','FontSize',18)
xlabel('Time [sec]','FontSize',16)
ylabel('Curveture [rad/m]','FontSize',16)
set(gca,'FontSize',14)

figure(6)
subplot(2,1,1)
plot(time_elapsed,data_mat(:,11),'k','LineWidth',2);
title('Distance between the user and the Kinect','FontSize',18)
xlabel('Time [sec]','FontSize',16)
ylabel('Distance [m]','FontSize',16)
set(gca,'FontSize',14)
subplot(2,1,2)
plot(time_elapsed,data_mat(:,1),'k','LineWidth',2);
title('Translational velocity set to the robot','FontSize',18)
xlabel('Time [sec]','FontSize',16)
ylabel('Velocity [m/s]','FontSize',16)
set(gca,'FontSize',14)

figure(7)
plot(time_elapsed,-data_mat(:,9)*pi/180,'k--','LineWidth',1)
hold on
plot(time_elapsed,-data_mat(:,10)*pi/180,'k','LineWidth',2);
title('Steering angle','FontSize',18)
xlabel('Time [sec]','FontSize',16)
ylabel('Angle [rad]','FontSize',16)
legend('Real steering angle','Normalized steering angle')
set(gca,'FontSize',14)


% figure(8)
% plot(time_elapsed,-data_mat(:,10),'LineWidth',2);
% title('Set rotational velocity','FontSize',18);
% xlabel('Time [sec]','FontSize',16)
% ylabel('Velocity [rad/sec]','FontSize',16)
% set(gca,'FontSize',14)

figure(9)
% plot(time_elapsed,data_mat(:,5),'g','LineWidth',1);
% hold on
% plot(time_elapsed,-data_mat(:,6),'m','LineWidth',1);
%plot(time_elapsed,data_mat(:,7),'b','LineWidth',3);
xlabel('Time [sec]','FontSize',16)
ylabel('Average error [Pixel]','FontSize',16)
set(gca,'FontSize',14)
plotyy(time_elapsed,data_mat(:,7),time_elapsed,data_mat(:,8));
[AX,H1,H2] = plotyy(time_elapsed,data_mat(:,7),time_elapsed,data_mat(:,8));
set(get(AX(1),'Ylabel'),'String','Average error [Pixel]','FontSize',16)
set(get(AX(2),'Ylabel'),'String','Pan angle [degree]','FontSize',16) 
%yylabel('Pan angle [degree]','FontSize',16)
set(gca,'FontSize',14)
legend('Average error','Pan angle')
title('Average error and pan steering_angle(\lambda=10)','FontSize',16)
xlabel('Time [sec]','FontSize',16)
% subplot(2,2,2)
% plot(time_elapsed,-data_mat(:,6),'k','LineWidth',2);
% title('Right error','FontSize',18);
% xlabel('Time [sec]','FontSize',16)
% ylabel('Error [Pixel]','FontSize',16)
% set(gca,'FontSize',14)
% subplot(2,2,3)
% plot(time_elapsed,data_mat(:,7),'k','LineWidth',2);
% title('Average error');
% xlabel('Time [sec]','FontSize',16)
% ylabel('Error [Pixel]','FontSize',16)
% set(gca,'FontSize',14)
% subplot(2,2,4)
% plot(time_elapsed,data_mat(:,8),'k','LineWidth',2);
% title('Pan angle','FontSize',18);
% xlabel('Time [sec]','FontSize',16)
% ylabel('Error [Pixel]','FontSize',16)
% set(gca,'FontSize',14)

Curveture_Variance = var(Curveture_set)

Time_Duration = data_mat(size(data_mat,1),20) - data_mat(i,20)

command_no = 0;
rot_mins=imregionalmin(R_temp(:,4));
for i=2:size(rot_mins,1)
    if rot_mins(i,1)~= rot_mins(i-1,1)
        command_no = command_no + 1;
    end
end
rot_maxs=imregionalmax(R_temp(:,4));
for i=2:size(rot_mins,1)
    if rot_mins(i,1)~= rot_mins(i-1,1)
        command_no = command_no + 1;
    end
end
command_no = 1 + command_no/2
