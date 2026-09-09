% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

% Simulation Interface

close all;
clear all;
clc;

%% static functions
addpath('.\functions')

%% parameter settings
simstate                 = [];    % Global struct containg all the interior states
simstate.sensor          = [];    % Sensor Information
simstate.robot           = [];    % Roboter Position and properties
simstate.particlefilter  = [];    % Particle Filter
simstate.enviroment      = [];    % Properties of the enviroment

%% Sensor settings
simstate.sensor.maxrange              = 2.5;     % Range of distance sensor
simstate.sensor.std_sensor            = 0.4;

%% Robot settings
simstate.robot.waypoints              = [4   2.5;
                                         5   3  ;
                                         6   4.5;
                                         2   4.5;
                                         1.5 5.2;
                                         3.4 4.7;
                                         9.5 4.5;
                                         6.5 4.5;
                                         4.9 4]'; % waypoints for the robot navigation
simstate.robot.velocity               = 0.4;   % forward velocity
simstate.robot.dt                     = 0.1;   % timestep
simstate.robot.wheelbase              = 0.2;   % Wheelbase
simstate.robot.number_loops           =   2;   % number of loops (for future use)
simstate.robot.iwp                    =   1;   % index to first waypoint
simstate.robot.position               =  [];   % true pose of the robot
simstate.robot.minD                   = 0.3;   % minD - minimum distance to current waypoint before switching to next
simstate.robot.rateG                  = 30*pi/180; % rateG - max steering rate (rad/s)
simstate.robot.maxG                   = 30*pi/180; % maxG - max steering steering_angle(rad)
simstate.robot.G                      = 0;      % current steering angle
a                                     = [0.05 0.02 0.02 0.05 0.01 0.08]; % motion model Noise Parameters % 

%% Particle Filter settings
simstate.particlefilter.num_particles =  200;    % Number of particles
simstate.particlefilter.range         =  1:simstate.particlefilter.num_particles; % for visualization of particles
simstate.particlefilter.flag          =  1;    % Particle filter on;

%% generation of the map
simstate     = build_world(simstate); % Map creation
simstate     = show_world(simstate);  % Map visualization

disp('initialization simulation');

%% initial position
simstate.robot.position = [2.5;2.5;0];
% kidnap position
simstate.robot.kidnaped = [2.5;1.5;-pi/8];
% handle to position
simstate.robot.handle       = rectangle('Position',[simstate.robot.position(1)-0.05 simstate.robot.position(2)-0.05 0.1 0.1],'Curvature',[1,1],'FaceColor','g');
simstate.robot.handle       = rectangle('Position',[simstate.robot.kidnaped(1)-0.05 simstate.robot.kidnaped(2)-0.05 0.1 0.1],'Curvature',[1,1],'FaceColor','g');

%% Set initial partikels
if(simstate.particlefilter.flag)
    % particle position
    for i =1:simstate.particlefilter.num_particles
        simstate.particlefilter.particles.position(1,i)  = rand*simstate.enviroment.boundaryxr;
        simstate.particlefilter.particles.position(2,i)  = rand*simstate.enviroment.boundaryyu;
        simstate.particlefilter.particles.position(3,i)  = rand*2*pi;
        % at the start uniform distribution
        simstate.particlefilter.particles.probability(i) = 1/simstate.particlefilter.num_particles;
    end
    simstate.particlefilter.particles.handlet = text(simstate.particlefilter.particles.position(1,:),simstate.particlefilter.particles.position(2,:),num2str(simstate.particlefilter.range(:)),'color',[.7 .7 .7]);
    simstate.particlefilter.particles.handlep = plot(simstate.particlefilter.particles.position(1,:),simstate.particlefilter.particles.position(2,:),'*k');
    simstate.particlefilter.particles.handlee = rectangle('Position',[0 0 0.001 0.001],'Curvature',[1,1],'FaceColor','y'); % init dummy
end %if

%% visualization of robot waypoints
for i=1:size(simstate.robot.waypoints,2)
    text(simstate.robot.waypoints(1,i),simstate.robot.waypoints(2,i),num2str(i),'color','g'); % visualize waypoints
end
drawnow;

%% navigation and particle filter
last_r =[]; last_rm =[]; last_rrb =[]; last_mr = [];
disp('simulation running');
l=0;
while simstate.robot.iwp ~= 0
    l=l+1;
    % deleting handels to drawings | try for the case not initialised
    delete_graphics_handles(last_r,last_rm,last_rrb,last_mr)

    % compute steering and waypoint for the robot navigation ( velocity
    % remains constant during the simulation)
    [simstate] = compute_steering(simstate);

    % perform loops: if final waypoint reached, go back to first
    if simstate.robot.iwp==0 && simstate.robot.number_loops > 1
        simstate.robot.iwp=1;
        simstate.robot.number_loops= simstate.robot.number_loops-1;
        simstate.robot.position = simstate.robot.kidnaped;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%% MOTION MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % without noise currently
    % new position of the robot
    simstate.robot.position = sample_motion_model(simstate.robot.position,simstate,a);%+++++++++++++++++++++++++++++++++++++++++++
    
    % visualize current pose | Handle to current pose
    last_r = rectangle('Position',[simstate.robot.position(1)-0.05 simstate.robot.position(2)-0.05 0.1 0.1],'Curvature',[1,1],'FaceColor','b');

    % new positions of partciles
    if(simstate.particlefilter.flag)
        for i =1:simstate.particlefilter.num_particles%+++++++++++++++++++++++++++++++++++++++++++
            simstate.particlefilter.particles.position(1:3,i) = sample_motion_model(simstate.particlefilter.particles.position(:,i),simstate,a);%+++++++++++++++++++++++++++++++++++++++++++
        end;%+++++++++++++++++++++++++++++++++++++++++++
        delete(simstate.particlefilter.particles.handlet)
        delete(simstate.particlefilter.particles.handlep)
        simstate.particlefilter.particles.handlet = text(simstate.particlefilter.particles.position(1,:),simstate.particlefilter.particles.position(2,:),num2str(simstate.particlefilter.range(:)),'color',[.7 .7 .7]);
        simstate.particlefilter.particles.handlep = plot(simstate.particlefilter.particles.position(1,:),simstate.particlefilter.particles.position(2,:),'*k');
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%% SENSOR MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % get observations for robot
    [simstate.robot.measurement,rangebearing]  =  ray_casting(simstate.robot.position,simstate);
    z_t_star = simstate.robot.measurement(1,:); % measurements of the Robot
             % simstate.robot.measurement(2,:); % directions of the sensors (you don't need to use)
             
             % fill sample measurements to make them ralistic
             
             
    [last_rm,last_rrb] = drawmeasurements(simstate.robot.position,simstate.robot.measurement,3,'m',rangebearing); % handle to graphic object (draw red lines)
    % get observations for particles (ray casting)
    if(simstate.particlefilter.flag) % if particlefilter is on
        for i =1:simstate.particlefilter.num_particles
            [simstate.particlefilter.particles.measurement(:,:,i)]  =  ray_casting(simstate.particlefilter.particles.position(1:3,i)',simstate);
            %last_mr(i) = drawmeasurements(simstate.particlefilter.particles.position(1:3,i)',simstate.particlefilter.particles.measurement(:,:,i),1,'k'); % handle to graphic object
        end

%         for i = 1:simstate.particlefilter.num_particles
%             simstate.particlefilter.particles.probability(i) =  0;   % fill !!!!!!!!!!!!!!!!!!
%         end

            
        simstate.particlefilter.particles.probability = beam_range_finder_model(simstate);%+++++++++++++++++++++++++++++++++++++++++++
        prb(l,:)=simstate.particlefilter.particles.probability;%+++++++++++++++++++++++++++++++++++++++++++
        pm(l)=max(prb(l,:));%+++++++++++++++++++++++++++++++++++++++++++

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%  find Center Particle %%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%% nothing to do... %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        distanceBetweenParticles = euclideanDistanceMatrix(simstate.particlefilter.particles.position(1:2,:),Inf); % calculate the diatance between the objectives of all individuums of the elits
        [columMins,rowsOfMaxs] = min(distanceBetweenParticles); % look for the one with the smallest distace to an other member (row wise)
        medi=median(columMins);
        center=simstate.particlefilter.particles.position(1:2,:);
        center = center(:,columMins<medi); % delete 50% of particles
        center=mean(center,2);
        delete(simstate.particlefilter.particles.handlee)
        simstate.particlefilter.particles.handlee=rectangle('Position',[center(1)-0.05 center(2)-0.05 0.1 0.1],'Curvature',[1,1],'FaceColor','y');
        
   
    
        
    
    
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%  Resampling  %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
       % simstate.particlefilter.particles.position(1:3,:); % please Fill new
        %simstate.particlefilter.particles.probability; % please Fill new

    simstate.particlefilter.particles.position=resampling(simstate);%+++++++++++++++++++++++++++++++++++++++++++
    
    end

    drawnow;
    pause(0.01);

end


                                                 

                                                 