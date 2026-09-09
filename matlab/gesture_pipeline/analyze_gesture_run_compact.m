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


curveture = rotational_vel_set ./ translational_vel_set;
for i=1:size(curveture,1)
    if abs(curveture(i))==inf || isnan(curveture(i))
        curveture(i)=0;
    end
end

Time=data_mat(size(data_mat,1),20)
Tvel_Mean(1:size(translational_vel_set,1),1)=mean(translational_vel_set);
TMean=mean(translational_vel_set)
Rvel_Mean(1:size(rotational_vel_set,1),1)=mean(rotational_vel_set);
RMean=mean(rotational_vel_set)
curveture_Mean(1:size(curveture,1),1) = mean(curveture);
CMean=mean(curveture)
curveture_Var = var(curveture)

figure(1)
plot(time_elapsed,translational_vel_set,'k','LineWidth',2)
hold on
plot(time_elapsed,Tvel_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',16)
ylabel('Translational velocity [m/s]','FontSize',16)
legend('Velocity','Average')
set(gca,'FontSize',14)
figure(2)
plot(time_elapsed,rotational_vel_set,'k','LineWidth',2)
hold on
plot(time_elapsed,Rvel_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',16)
ylabel('Rotational velocity [m/s]','FontSize',16)
legend('Velocity','Average')
set(gca,'FontSize',14)
figure(3)
plot(time_elapsed,curveture,'k','LineWidth',2)
hold on
plot(time_elapsed,curveture_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',16)
ylabel('Curveture [rad/m]','FontSize',16)
legend('Curveture','Average')
set(gca,'FontSize',14)
figure(4)
subplot(2,1,1)
plot(time_elapsed,distance_m)
subplot(2,1,2)
plot(time_elapsed,translational_vel_set)

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