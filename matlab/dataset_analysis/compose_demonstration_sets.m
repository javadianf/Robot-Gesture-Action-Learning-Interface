% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

 folders = dir('CorridorFollowing*');
 Currentpath = pwd;
  for nIndexfolders = 1 : length(folders)
     foldername = folders(nIndexfolders).name;
     cd(foldername);
     files = dir('*.mat');
     load(files(end).name);
     save(['<LOCAL_PATH>\' foldername '.mat'],'Time_elapsed','Vel_array','omega_array');
     cd(Currentpath);
 end
 