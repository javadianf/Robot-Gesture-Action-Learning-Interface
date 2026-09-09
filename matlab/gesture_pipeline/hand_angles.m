% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [ left_angle, right_angle ] = hand_angles( head, torso, left_hand,right_hand)
%method to calculate the rotation angle from distances
%   Detailed explanation goes here
t1= head - torso;
t2= left_hand - torso;
t3= right_hand - torso;
d1 =  norm(cross(t1,t2))/ norm (t1);
d2 =  norm(cross(t1,t3))/ norm (t1);

hand_distance = 0.5 * sqrt((left_hand(1)-right_hand(1))^2+(left_hand(2)-right_hand(2))^2);

left_angle = acos(hand_distance / d1);
right_angle= acos(hand_distance / d2);

end

