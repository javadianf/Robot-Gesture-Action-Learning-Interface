% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function p= simulate_laser_lines(simstate,i)
% compute set of line segments for laser range-bearing measurements
if isempty(simstate.particlesim.rangebearing{i}), p=[]; return, end
len= size(simstate.particlesim.rangebearing{i},2);
lnes(1,:)= zeros(1,len)+ simstate.robot_groundtruth(1);
lnes(2,:)= zeros(1,len)+ simstate.robot_groundtruth(2);
lnes(3:4,:)= transformtoglobal([simstate.robot.rangebearing(1,:).*cos(simstate.robot.rangebearing(2,:)); simstate.robot.rangebearing(1,:).*sin(simstate.robot.rangebearing(2,:))], simstate.robot_groundtruth);
p= line_plot_conversion (lnes);  
