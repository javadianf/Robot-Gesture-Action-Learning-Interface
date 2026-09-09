% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [simstate]= get_observations(simstate)

%% init
simstate.robot.rangebearing = [];
%% Detekt landmarks in the sonar range
simstate= get_visible_landmarks(simstate);
%% Extrakt distance and bearing
simstate = compute_range_bearing(simstate);



function [simstate]= get_visible_landmarks(simstate)
% Select set of landmarks that are visible within vehicle's semi-circular field-of-view
dx = simstate.enviroment.points(1,:) - simstate.robot_groundtruth(1);
dy = simstate.enviroment.points(2,:) - simstate.robot_groundtruth(2);
phi= simstate.robot_groundtruth(3);

% incremental tests for bounding semi-circle
ii= find(abs(dx) < simstate.maxrange & abs(dy) < simstate.maxrange ... % bounding box
      & (dx*cos(phi) + dy*sin(phi)) > 0 ...                            % bounding line
      & (dx.^2 + dy.^2) < simstate.maxrange^2);                        % bounding circle
% Note: the bounding box test is unnecessary but illustrates a possible speedup technique
% as it quickly eliminates distant points. Ordering the landmark set would make this operation
% O(logN) rather that O(N).
simstate.enviroment.pointsdetected = simstate.enviroment.points(:,ii);


function simstate= compute_range_bearing(simstate)
% Compute exact observation
dx= simstate.enviroment.pointsdetected(1,:) - simstate.robot_groundtruth(1);
dy= simstate.enviroment.pointsdetected(2,:) - simstate.robot_groundtruth(2);
phi= simstate.robot_groundtruth(3);
simstate.robot.rangebearing = [sqrt(dx.^2 + dy.^2);
    atan2(dy,dx) - phi];
    