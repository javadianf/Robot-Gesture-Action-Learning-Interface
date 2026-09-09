% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function newposition= sample_motion_model(position,simstate,a)

% You have
dt = simstate.robot.dt;            % timestep
%simstate.robot.wheelbase     % Wheelbase
v = simstate.robot.velocity;      % velocity
%simstate.robot.G             % steering angle !!! not rotation Velocity
w = simstate.robot.omega;         % rotation velocity
a1=a(1);
a2=a(2);
a3=a(3);
a4=a(4);
a5=a(5);
a6=a(6);

x=position(1);
y=position(2);
t=position(3);

vhat =  v + sample_normal_distribution (a1 * (v^2) + a2 * (w^2));
what =  w + sample_normal_distribution (a3 * (v^2) + a4 * (w^2));
yhat =  sample_normal_distribution (a5 * (v^2) + a6 * (w^2));

xp = x - (vhat/what)*sin(t) + (vhat/what)*sin(t + what * dt);
yp = y + (vhat/what)*cos(t) - (vhat/what)*cos(t + what * dt);
tp = t +what * dt + yhat * dt;


newposition(1) = xp; 
newposition(2) = yp; 
newposition(3) = tp; 

 