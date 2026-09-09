% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [handle1 handle2] = drawmeasurements(position,measurement,line_size,color,rangebearing)

if nargin < 5
 plines         = make_laser_lines(measurement,position);  % show the measurements;
 if (size(plines,1) >0)
 handle1 = plot(plines(1,:),plines(2,:),color,'erasemode','normal','linewidth',line_size); % observations
 handle2 = [];
 end
else
 plines         = make_laser_lines(measurement,position);  % show the measurements;
 if (size(plines,1) >0)
 handle1 = plot(plines(1,:),plines(2,:),color,'erasemode','normal','linewidth',line_size); % observations
 plines         = make_laser_lines(rangebearing,position);  % show the measurements;
 end
 if (size(plines,1) >0)
 handle2 = plot(plines(1,:),plines(2,:),'r','erasemode','normal'); % observations
 end
end