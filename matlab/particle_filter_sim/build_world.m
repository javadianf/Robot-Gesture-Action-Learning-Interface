% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function simstate = build_world(simstate)

%dichte
simstate.enviroment.dichte = 0.1; 
simstate.enviroment.boundaryxl =  0;
simstate.enviroment.boundaryxr = 10;
simstate.enviroment.boundaryyl =  0;
simstate.enviroment.boundaryyu =  6;
%left wall
left_wall_y  = 0:simstate.enviroment.dichte:6;
left_wall_x  = 0*ones(1,length(left_wall_y ));

% right wall
right_wall_y = 0:simstate.enviroment.dichte:6;
right_wall_x = 10*ones(1,length(right_wall_y));

% lower wall
lower_wall_x = 0:simstate.enviroment.dichte:10;
lower_wall_y = 6*ones(1,length(lower_wall_x));

% upper wall
upper_wall_x = 0:simstate.enviroment.dichte:10;
upper_wall_y = 0*ones(1,length(upper_wall_x));

% first itermediate wall horizontal
firstinter_wall_x = [0:simstate.enviroment.dichte:3 4:simstate.enviroment.dichte:7 8:simstate.enviroment.dichte:10];
firstinter_wall_y =  2*ones(1,length(firstinter_wall_x));

% second itermediate wall horizontal
secondinter_wall_x = [0:simstate.enviroment.dichte:4 7:simstate.enviroment.dichte:10];
secondinter_wall_y =  3.5*ones(1,length(secondinter_wall_x));

% first itermediate wall verticaL
firstinter_wall_vy = 0:simstate.enviroment.dichte:2;
firstinter_wall_vx = 5.5*ones(1,length(firstinter_wall_vy));

% second itermediate wall verticaL
secondinter_wall_vy = [3.5:simstate.enviroment.dichte:4 5:simstate.enviroment.dichte:6];
secondinter_wall_vx = 4*ones(1,length(secondinter_wall_vy));

% third itermediate wall verticaL
thirdinter_wall_vy = [3.5:simstate.enviroment.dichte:4 5:simstate.enviroment.dichte:6];
thirdinter_wall_vx = 7*ones(1,length(thirdinter_wall_vy ));

simstate.enviroment.points = [left_wall_x right_wall_x lower_wall_x upper_wall_x firstinter_wall_x secondinter_wall_x firstinter_wall_vx secondinter_wall_vx thirdinter_wall_vx;...
                              left_wall_y right_wall_y lower_wall_y upper_wall_y firstinter_wall_y secondinter_wall_y firstinter_wall_vy secondinter_wall_vy thirdinter_wall_vy];

