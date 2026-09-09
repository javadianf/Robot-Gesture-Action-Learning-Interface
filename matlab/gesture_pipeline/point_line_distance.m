% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [ d ] = point_line_distance( point1,point2,point )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
t1= point1 - point2;
t2= point - point2;
d =  norm(cross(t1,t2))/ norm (t1);

end

