% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [simstate]= compute_steering(simstate)
%
% INPUTS:
%   x - true position
%   wp - waypoints
%   iwp - index to current waypoint
%   minD - minimum distance to current waypoint before switching to next
%   G - current steering angle
%   rateG - max steering rate (rad/s)
%   maxG - max steering steering_angle(rad)
%   dt - timestep
%
% OUTPUTS:
%   G - new current steering angle
%   iwp - new current waypoint

x     = simstate.robot.position;
wp    = simstate.robot.waypoints;
iwp   = simstate.robot.iwp ;
minD  = simstate.robot.minD;
G     = simstate.robot.G ; 
rateG = simstate.robot.rateG;
maxG  = simstate.robot.maxG;
dt    = simstate.robot.dt;

% determine if current waypoint reached
cwp= wp(:,iwp);
d2= (cwp(1)-x(1))^2 + (cwp(2)-x(2))^2;
if d2 < minD^2
    iwp= iwp+1; % switch to next
    if iwp > size(wp,2) % reached final waypoint, flag and return
        simstate.robot.iwp =0;
        return;
    end    
    cwp= wp(:,iwp); % next waypoint
end

% compute change in G to point towards current waypoint
deltaG= pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - x(3) - G);

% limit rate
maxDelta= rateG*dt;
if abs(deltaG) > maxDelta
    deltaG= sign(deltaG)*maxDelta;
end

% limit angle
G= G+deltaG;
if abs(G) > maxG
    G= sign(G)*maxG;
end

simstate.robot.iwp  = iwp;
simstate.robot.G    =   G;

simstate.robot.omega = pi_to_pi(simstate.robot.velocity*sin(simstate.robot.G)/simstate.robot.wheelbase);

