% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [ joint_positions ] = joint_parse(name)
% A function to parse texted bag files
%   Detailed explanation goes here
mat = textread(name, '', 'delimiter', ',','emptyvalue', NaN);
%joint_postions_mixed=(mat(:,4:6));
frame_nom=size(mat,1);
count=1;
for i=1:15:(floor(frame_nom/15))*15
    %nead
    joint_positions(count,:,1)=mat(i,4:6);
    %neck
    joint_positions(count,:,2)=mat(i+1,4:6);
    %torso
    joint_positions(count,:,3)=mat(i+2,4:6);
    %left_sholder
    joint_positions(count,:,4)=mat(i+3,4:6);
    %left_elbow
    joint_positions(count,:,5)=mat(i+4,4:6);
    %left_hand
    joint_positions(count,:,6)=mat(i+5,4:6);
    %right_sholder
    joint_positions(count,:,7)=mat(i+6,4:6);
    %right_elbow
    joint_positions(count,:,8)=mat(i+7,4:6);
    %right_hand
    joint_positions(count,:,9)=mat(i+8,4:6);
    %left_hip
    joint_positions(count,:,10)=mat(i+9,4:6);
    %left_knee
    joint_positions(count,:,11)=mat(i+10,4:6);
    %left_foot
    joint_positions(count,:,12)=mat(i+11,4:6);
    %right_hip
    joint_positions(count,:,13)=mat(i+12,4:6);
    %right_knee
    joint_positions(count,:,14)=mat(i+13,4:6);
    %right_foot
    joint_positions(count,:,15)=mat(i+14,4:6);
    count=count+1;
end
figure(1)
hold on
for i=1:15
    %plot3(joint_positions(:,1,i),joint_positions(:,2,i),joint_positions(:,3,i),'*');
    plot3(joint_positions(:,1,i),joint_positions(:,2,i),joint_positions(:,3,i));
end

end

