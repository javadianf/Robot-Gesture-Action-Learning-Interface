% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [ joint_positions ] = joint_parse_alt_log(name)
% A function to parse texted bag files
%   Detailed explanation goes here
%mat = textread(name, '', 'delimiter', ',','emptyvalue', NaN);
[mat(:,1),mat(:,2),mat(:,3)]=textread(name, '%f %f %f');
%joint_postions_mixed=(mat(:,4:6));
frame_nom=size(mat,1);
count=1;
for i=1:5:(floor(frame_nom/5))*5
    %nead
    joint_positions(count,:,1)=mat(i,1:3);
    %neck
    joint_positions(count,:,5)=mat(i+4,1:3);
    %torso
    joint_positions(count,:,4)=mat(i+3,1:3);
    
    %left_hand
    joint_positions(count,:,2)=mat(i+1,1:3);
    
    %right_hand
    joint_positions(count,:,3)=mat(i+2,1:3);
    
    count=count+1;
end
figure(1)
hold on
for i=1:5
    %plot3(joint_positions(:,1,i),joint_positions(:,2,i),joint_positions(:,3,i),'*');
    plot3(joint_positions(:,1,i),joint_positions(:,2,i),joint_positions(:,3,i));
end

end

