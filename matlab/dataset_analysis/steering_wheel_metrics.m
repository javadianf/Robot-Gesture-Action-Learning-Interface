% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

clear all
clc
close all
[Names Path Ind] = uigetfile('.mat','Select','MultiSelect','on')
Data_mat=zeros(size(Names,2),6);
for i=1:size(Names,2)
    load([Path Names{i}])
Data_mat(i,1)=tvel;
Data_mat(i,2)=rvel;
Data_mat(i,3:5)=Current_time;
Data_mat(i,6)=(Data_mat(i,5)-Data_mat(1,5))+60*(Data_mat(i,4)-Data_mat(1,4))+3600*(Data_mat(i,3)-Data_mat(1,3));
end 

i=size(Data_mat,1)-1;
while Data_mat(i,1) == Data_mat(i+1,1) && Data_mat(i,2) == Data_mat(i+1,2)
    i=i-1;
end
Data_mat=Data_mat(1:i,:);
i=2;
while Data_mat(i,1) == 0 && Data_mat(i,2) == 0
    i=i+1;
end
time_bias=Data_mat(i,6);
Data_mat=Data_mat(i:size(Data_mat,1),:);
Data_mat(:,6)=Data_mat(:,6)-time_bias;

curveture = Data_mat(:,2) ./ Data_mat(:,1);
for i=1:size(curveture,1)
    if abs(curveture(i))==inf || isnan(curveture(i))
        curveture(i)=0;
    end
end

Time=Data_mat(size(Data_mat,1),6)
Tvel_Mean(1:size(Data_mat,1),1)=mean(Data_mat(:,1));
TMean=mean(Data_mat(:,1))
Rvel_Mean(1:size(Data_mat,1),1)=mean(Data_mat(:,2));
RMean=mean(Data_mat(:,2))
curveture_Mean(1:size(curveture,1),1) = mean(curveture);
CMean=mean(curveture)
curveture_Var = var(curveture)
%time_elapsed=Data_mat(:,6);

figure(1)
plot(Data_mat(:,6),Data_mat(:,1),'k','LineWidth',2)
hold on
plot(Data_mat(:,6),Tvel_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',20)
ylabel('Translational velocity [m/s]','FontSize',20)
legend('Velocity','Average')
set(gca,'FontSize',16)
figure(2)
plot(Data_mat(:,6),Data_mat(:,2),'k','LineWidth',2)
hold on
plot(Data_mat(:,6),Rvel_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',20)
ylabel('Rotational velocity [rad/s]','FontSize',20)
legend('Velocity','Average')
set(gca,'FontSize',16)
figure(3)
plot(Data_mat(:,6),curveture,'k','LineWidth',2)
hold on
plot(Data_mat(:,6),curveture_Mean,'k-.','LineWidth',0.5)
xlabel('Time [sec]','FontSize',20)
ylabel('Curveture [rad/m]','FontSize',20)
legend('Curveture','Average')
set(gca,'FontSize',16)

command_no = 0;
rot_mins=imregionalmin(Data_mat(:,2));
for i=2:size(rot_mins,1)
    if rot_mins(i,1)~= rot_mins(i-1,1)
        command_no = command_no + 1;
    end
end
rot_maxs=imregionalmax(Data_mat(:,2));
for i=2:size(rot_maxs,1)
    if rot_maxs(i,1)~= rot_maxs(i-1,1)
        command_no = command_no + 1;
    end
end
command_no = 1 + command_no/2
 