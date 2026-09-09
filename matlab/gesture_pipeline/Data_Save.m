% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function Data_Save(rvel, tvel, camview, SonarData , Om_IMG)
% Appending new data to end of the Saved Data
load(Mat_Add); % Loading data from last itterations
new_Tag = datestr(now,0); % Finding new time tag
DataSave.Tag = [DataSave.Tag ; new_Tag]; % Adding new Tag to end of the Saved Data
DataSave.Vel = [DataSave.Vel ; [rvel, tvel, camview]]; % Adding new Velocity data to end of the Saved Data
DataSave.Son = [DataSave.Son ; SonarData]; % Adding new Sonar data to end of the Saved Data
save(Mat_Add, 'DataSave') % Saving new Saved data Matrix
% End of Data savin

% Saving image from Omni Camera
global IMG_Add
global Image_Name
Image_Tag = datestr(datenum(new_Tag),30); % Providing date and time tag for image
Image_Name = strcat('IMG', Image_Tag); % Name of Image at this itteration
assignin('caller', Image_Name, Om_IMG);
IMG_Add = strcat(Saving_Folder, '\', Image_Name, '.mat');