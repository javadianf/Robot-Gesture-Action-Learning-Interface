% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

clear all;
close all;
% reply = input('Please enter the file name(TXT): ', 's');
% data_mat = textread(reply,'%f','headerlines',9);
[Names Path Ind] = uigetfile('.txt','Select','MultiSelect','on')
for k=1:size(Names,2)
     %load([Path Names{k}])

DELIMITER = ' ';
HEADERLINES = 9;

% Import the file
data_mat_ = importdata(eval('Names{k}'), DELIMITER, HEADERLINES);
% data_mat_ = importdata(reply, DELIMITER, HEADERLINES);
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

% Time=data_mat(size(data_mat,1),20)
Time=data_mat(:,20);
Tvel_Mean(1:size(translational_vel_set,1),1)=mean(translational_vel_set);
TMean=mean(translational_vel_set);
Rvel_Mean(1:size(rotational_vel_set,1),1)=mean(rotational_vel_set);
RMean=mean(rotational_vel_set);
curveture_Mean(1:size(curveture,1),1) = mean(curveture);
CMean=mean(curveture);
curveture_Var = var(curveture);


   

            if(k<10)

            zeros_ = '00';

            elseif(k>=10 && k<100)
 
              zeros_ = '0';

            elseif(k>=100)

              zeros_ = '';

            end


q=zeros(size(data_mat,1),4);
q(:,1)=data_mat(:,14); % omega
q(:,4)=-10*data_mat(:,15); % z
R=zeros(size(data_mat,1),4);
R = SpinCalc('QtoEA321',q,0,0);
% [R(:,1) R(:,2) R(:,3)] = quat2angle(q);
R(:,4)=R(:,3)*(pi/180);
Orientation = R(:,4);% radian

Full_path = 'C:\<USER>\Downloads\dataSet\Gesture Recognition HRI\';
% Topic = ['demo_door_passing_1                      ';'Time                                 ';'Rotational velocity set              ';'Rotational velocity read from robot  ';'Translational velocity set           ';'Translational velocit read from robot';'Left error                           ';'Right error                          ';'Average_error                        ';'Pan angle                            '; 'Curvature                            '  ; 'X Localization                       '; 'Y Localization                       '  ;  'X odometry                           ' ; 'Y odometry                           ' ; 'Orientation                          '; 'Steering angle                       ' ; 'Steering angle normalized            '; 'Distance                             '   ];
% % 
Topic = ['demo_corridor_following_' zeros_ num2str(k)];  

parameters = ['Time                                 '
;'Rotational velocity set              '
;'Rotational velocity read from robot  '
;'Translational velocity set           '
;'Translational velocit read from robot'];




% save([Full_path 'Time' '.mat'],'Time');
% save([Full_path 'Rotational_velocity_set' '.mat'],'rotational_vel_set');
% save([Full_path 'Rotational_velocity_read_from_robot' '.mat'],'rotational_vel_robot');
% save([Full_path 'Translational_velocity_set' '.mat'],'translational_vel_set');
% save([Full_path 'Distance' '.mat'],'distance_m');
% save([Full_path 'Translational_velocity_read_from_robot' '.mat'],'translational_vel_robot');
% save([Full_path 'Left_error' '.mat'], 'left_error');
% save([Full_path 'Right_error' '.mat'], 'right_error');
% save([Full_path 'Average_error' '.mat'], 'average_error');
% save([Full_path 'Pan_angle' '.mat'], 'pan_angle');
% save([Full_path 'Curvature' '.mat'], 'curveture');
% save([Full_path 'X_Localization' '.mat'], 'local_pose_x');
% save([Full_path 'Y_Localization' '.mat'], 'local_pose_y');
% save([Full_path 'X_odometry' '.mat'], 'robot_pose_x');
% save([Full_path 'Y_odometry' '.mat'], 'robot_pose_y');
% save([Full_path 'Orientation' '.mat'], 'Orientation');
% save([Full_path 'Steering_angle' '.mat'], 'rotation_real');
% save([Full_path 'Steering_angle_normalized' '.mat'], 'rotation_normalized');

% save([Full_path 'demo_door_passing_23' '.mat'], 'Topic' , 'Time' , 'rotational_vel_set' ,'rotational_vel_robot' ,  'translational_vel_set', 'translational_vel_robot' , 'left_error' ,'right_error',  'average_error' , 'pan_angle', 'curveture' , 'local_pose_x' , 'local_pose_y' , 'robot_pose_x' ,'robot_pose_y' ,'Orientation' , 'rotation_real', 'rotation_normalized', 'distance_m');
save([Full_path 'demo_corridor_following_' zeros_ num2str(k) '.mat'], 'Topic', 'parameters' , 'Time' , 'rotational_vel_set' ,'rotational_vel_robot' ,  'translational_vel_set', 'translational_vel_robot');
end
