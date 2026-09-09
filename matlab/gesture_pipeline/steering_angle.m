% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [ angle_lines , angle_right , angle_left, mix_angle] = steering_angle( head, torso, neck, right_hand , left_hand)
%First method to calculate the rotation angle
%   head, torso, neck, left_hand,right_hand are points [x,y]
t1 = head - torso;
t2 =  right_hand- left_hand;

% angle_lines = mod((atan2(det([t1;t2]),dot(t1,t2))-(pi/2)),2*pi)*180/pi;
angle_lines = mod((atan2(det([t1;t2]),dot(t1,t2))-(pi/2)),2*pi);
if angle_lines>pi
    angle_lines = angle_lines - (2*pi);
end
angle_lines = angle_lines *(180/pi);

%angle_lines = angle_lines - 90;
% if angle_lines>180
%     angle_lines = angle_lines - 360;
% end

%a = mod(atan2(det([t1;t2]),dot(t1,t2)),2*pi);
%angle_lines = abs((a>pi/2)*pi-a);

%different method to calculate angles
t_right = right_hand - neck;
t_left = left_hand - neck;

angle_left = mod(atan2(det([t_left;t1]),dot(t_left,t1)),pi)*180/pi;
angle_right = mod(atan2(det([t1;t_right]),dot(t1,t_right)),pi)*180/pi;

angle_right = angle_right - 90;
angle_left = 90- angle_left;

mix_angle = max(abs(angle_right),abs(angle_left))/sign(angle_right);


end

