% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [simstate] = show_world(simstate)
simstate.enviroment.h = figure; % figure handle
plot(simstate.enviroment.points(1,:), simstate.enviroment.points(2,:),'r.','MarkerSize',4)
hold on;
for i = 1:length(simstate.enviroment.points(1,:))
    rectangle('Position',[simstate.enviroment.points(1,i)-simstate.enviroment.dichte/2,simstate.enviroment.points(2,i)-simstate.enviroment.dichte/2,simstate.enviroment.dichte,simstate.enviroment.dichte])
end
axis ([-1 11 -1 7])
title('MAP of the environment');
drawnow;
pause(0.5);