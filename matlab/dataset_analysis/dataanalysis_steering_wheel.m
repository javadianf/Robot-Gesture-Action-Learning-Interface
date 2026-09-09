% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

[Names Path Ind] = uigetfile('.mat','Select','MultiSelect','on')

Full_path = 'C:\<USER>\Downloads\dataSet\Steering Wheel\';
 
% for i=1:size(Names,2)

%     load([Path Names{i}])

%             if(i<10)

%             zeros_ = '00';

%             elseif(i>=10 && i<100)

 %               zeros_ = '0';

%             elseif(i>=100)

%               zeros_ = '';

%             end

%     save([Full_path 'file_' zeros_ num2str(i) '.mat'],'rvel','tvel','camview','SonarData','Current_time');
 
% end 

Rotational_velocity = zeros(size(Names,2),1);

Translational_velocity = zeros(size(Names,2),1);

Camera_View = zeros(size(Names,2),1);

Time = zeros(size(Names,2),1);

empty = zeros(size(Names,2),3);
for i=1:size(Names,2)

    load([Path Names{i}])

%             if(i<10)
% 
%             zeros_ = '00';
% 
%             elseif(i>=10 && i<100)
%  
%               zeros_ = '0';
% 
%             elseif(i>=100)
% 
%               zeros_ = '';
% 
%             end

Translational_velocity(i,1) = tvel;
Camera_View(i,1) = camview;
Rotational_velocity(i,1) = rvel;
empty(i,1:3) = Current_time;

Time(i,1) = (empty(i,3)-empty(1,3))+60*(empty(i,2)-empty(1,2))+3600*(empty(i,1)-empty(1,1));
end 
 save([Full_path 'demo_homing1.mat'],'Translational_velocity','Rotational_velocity','Time');
 
% load([Path Names{size(Names,2)}])
% 
% save([Full_path 'image_' zeros_ num2str(size(Names,2)) '.mat'],'Om_IMG','Current_time');    
