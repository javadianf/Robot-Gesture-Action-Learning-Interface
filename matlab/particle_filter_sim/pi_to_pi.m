% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function angle = pi_to_pi(angle)

%function angle = pi_to_pi(angle)
% Input: array of angles.
% Tim Bailey 2000

angle = mod(angle, 2*pi);

i=find(angle>pi);
steering_angle(i)=steering_angle(i)-2*pi;

i=find(angle<-pi);
steering_angle(i)=steering_angle(i)+2*pi;
