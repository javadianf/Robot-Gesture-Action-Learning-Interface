% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

 folders = dir('CorridorFollowing*');
 Currentpath = pwd;
  for nIndexfolders = 1 : length(folders)
     foldername = folders(nIndexfolders).name;
     cd(foldername);
     files = dir('*.mat');
     load(files(end).name);
     save(['d:\LaV_MoRF\Data\Demonstration_modes\UserStudies_Data\other demos\Data_18_October\Joystick\Composed\' foldername '.mat'],'Time_elapsed','Vel_array','omega_array');
     cd(Currentpath);
 end
 
