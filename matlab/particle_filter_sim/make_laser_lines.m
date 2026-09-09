% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function p= make_laser_lines(rangebearing, pose)

% compute set of line segments for laser range-bearing measurements
if isempty(rangebearing), p=[]; return, end
len= size(rangebearing,2);
lnes(1,:)= zeros(1,len)+ pose(1);
lnes(2,:)= zeros(1,len)+ pose(2);
lnes(3:4,:)= transformtoglobal([rangebearing(1,:).*cos(rangebearing(2,:)); rangebearing(1,:).*sin(rangebearing(2,:))], pose);
p= line_plot_conversion (lnes);  
