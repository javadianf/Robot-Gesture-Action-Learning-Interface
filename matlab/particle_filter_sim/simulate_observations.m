% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [simstate]= simulate_observations(simstate)
warning off;

for i =1:simstate.num_particles
 %% Detekt landmarks in the sonar range
 simstate= get_visible_landmarks(simstate,i);
 %% Extrakt distance and bearing
 simstate = compute_range_bearing(simstate,i);
end

   
function [simstate]= get_visible_landmarks(simstate,i)
% Select set of landmarks that are visible within vehicle's semi-circular field-of-view
dx = simstate.enviroment.points(1,:) - simstate.particle(1,i);
dy = simstate.enviroment.points(2,:) - simstate.particle(2,i);
phi= simstate.particle(3,i);

% incremental tests for bounding semi-circle
ii= find(abs(dx) < simstate.maxrange & abs(dy) < simstate.maxrange ... % bounding box
      & (dx*cos(phi) + dy*sin(phi)) > 0 ...                            % bounding line
      & (dx.^2 + dy.^2) < simstate.maxrange^2);                        % bounding circle
% Note: the bounding box test is unnecessary but illustrates a possible speedup technique
% as it quickly eliminates distant points. Ordering the landmark set would make this operation
% O(logN) rather that O(N).
simstate.particlesim{i} = simstate.enviroment.points(:,ii);


function simstate= compute_range_bearing(simstate,i)
% Compute exact observation
if (~isempty(simstate.particlesim{i}));
dx= simstate.particlesim{i}(1,:) - simstate.particle(1,i);
dy= simstate.particlesim{i}(2,:) - simstate.particle(2,i);
phi= simstate.particle(3,i);
simstate.particlesimrangebearing{i} = [sqrt(dx.^2 + dy.^2);
     atan2(dy,dx) - phi];
else
simstate.particlesimrangebearing{i} = [];
end
warning on;