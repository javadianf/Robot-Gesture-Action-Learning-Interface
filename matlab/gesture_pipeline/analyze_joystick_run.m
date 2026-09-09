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
%data_mat=data_mat(1:size(data_mat,1)-1,:);
i=2;
while data_mat(i,5) == 0 && data_mat(i,6) == 0
    i=i+1;
end
time_bias=data_mat(i,13);
data_mat=data_mat(i:size(data_mat,1),:);
data_mat(:,13)=data_mat(:,13)-time_bias;
rotational_vel_set = data_mat(:,6)*(180/pi);
rotational_vel_robot = data_mat(:,12)*(180/pi);
translational_vel_set = data_mat(:,5);
translational_vel_robot = data_mat(:,11);
local_pose_x = data_mat(:,1);
local_pose_y = data_mat(:,2);
local_quaternion_z = data_mat(:,3);
local_quaternion_w = data_mat(:,4);
robot_pose_x = data_mat(:,7);
robot_pose_y = data_mat(:,8);
robot_quaternion_z = data_mat(:,9);
robot_quaternion_w = data_mat(:,10);
time_elapsed = data_mat(:,13)/100000;
curveture = rotational_vel_set ./ translational_vel_set;
for i=1:size(curveture,1)
    if abs(curveture(i))==inf || isnan(curveture(i))
        curveture(i)=0;
    end
end

Time=data_mat(size(data_mat,1),13)
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
figure(2)
plot(time_elapsed,rotational_vel_set,'k','LineWidth',2)
hold on
plot(time_elapsed,Rvel_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',16)
ylabel('Rotational velocity [m/s]','FontSize',16)
legend('Velocity','Average')
figure(3)
plot(time_elapsed,curveture,'k','LineWidth',2)
hold on
plot(time_elapsed,curveture_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',16)
ylabel('Curveture [rad/m]','FontSize',16)
legend('Curveture','Average')

command_no = 0;
rot_mins=imregionalmin(rotational_vel_set);
for i=2:size(rot_mins,1)
    if rot_mins(i,1)~= rot_mins(i-1,1)
        command_no = command_no + 1;
    end
end
rot_maxs=imregionalmax(rotational_vel_set);
for i=2:size(rot_mins,1)
    if rot_mins(i,1)~= rot_mins(i-1,1)
        command_no = command_no + 1;
    end
end
command_no = 1 + command_no/2