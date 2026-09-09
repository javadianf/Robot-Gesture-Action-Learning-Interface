% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function xt=sample_motion_model_velocity (u,xi)

% INITIALIZATION ----------------------------------------------------------
dt = 0.001;

a1=0.05; a2=0.02;
a3=0.02; a4=0.05;
a5=0.01; a6=0.08;

v = u(1,1);
w = u(2,1);

x = xi(1,1);
y = xi(2,1);
t = xi(3,1);
%--------------------------------------------------------------------------
vhat =  v + sample_normal_distribution (a1 * (v^2) + a2 * (w^2));
what =  w + sample_normal_distribution (a3 * (v^2) + a4 * (w^2));
yhat =  sample_normal_distribution (a5 * (v^2) + a6 * (w^2));

xp = x - (vhat/what)*sin(t) + (vhat/what)*sin(t + what * dt);
yp = y + (vhat/what)*cos(t) - (vhat/what)*cos(t + what * dt);
tp = t +what * dt + yhat * dt;

xt=[xp,yp,tp]';