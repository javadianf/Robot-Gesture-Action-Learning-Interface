% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [measurement, rangebearing]  = ray_casting(position,simstate)

% 1) Get grid points in the sensor range
% 2) Calculate range and bearing
% 3) Map them into the sonar conics
% 4) Sample raw sensor reading sensor


%%  1) Get grid points in the sensor range
[pointsdetected]= get_visible_landmarks(position,simstate);
%% 2) Calculate range and bearing
rangebearing    = compute_range_bearing(position,pointsdetected);
%% 3) Map them into the sonar conics
measurement     = mapping(rangebearing);
%% 4) Sample raw sensor reading sensor
measurement     = sampleraw_sensor(measurement,simstate);

% 1)
function [pointsdetected] = get_visible_landmarks(position,simstate)
% Select set of landmarks that are visible within vehicle's semi-circular field-of-view
dx  = simstate.enviroment.points(1,:) - position(1);
dy  = simstate.enviroment.points(2,:) - position(2);
phi = position(3);
% incremental tests for bounding semi-circle
ii= find(abs(dx) < simstate.sensor.maxrange & abs(dy) < simstate.sensor.maxrange ... % bounding box
    & (dx*cos(phi) + dy*sin(phi)) > 0 ...                                          % bounding line
    & (dx.^2 + dy.^2) < simstate.sensor.maxrange^2);                               % bounding circle
% Note: the bounding box test is unnecessary but illustrates a possible speedup technique
% as it quickly eliminates distant points. Ordering the landmark set would make this operation
% O(logN) rather that O(N).
pointsdetected = simstate.enviroment.points(:,ii);

% 2)
function rangebearing = compute_range_bearing(position,pointsdetected)
% Compute exact observation
dx = pointsdetected(1,:) - position(1);
dy = pointsdetected(2,:) - position(2);
phi = position(3);
rangebearing = [sqrt(dx.^2 + dy.^2);
    pi_to_pi(atan2(dy,dx) - phi)];

% 3)
% simulate raw sonar readings by mapping all range and bearings to the grid
% elements into the sonar conics
function  measurement = mapping(rangebearing)
measurement       = zeros(2,8);
measurement(2,:)  = pi/16*[-7 -5 -3 -1 1 3 5 7];
map_measure  = ceil(rangebearing(2,:)/(pi/7)+pi);
for i = 0:7;
    index = find(map_measure == i);
    if (~isempty(index))
        value  = min(rangebearing(1,index));
        measurement(1,i+1) = value;
    end
end

% 4) sample sensor readings
function measurement     = sampleraw_sensor(measurement,simstate)
index = find(measurement(1,:) == 0);
measurement(1,index) = simstate.sensor.maxrange;
