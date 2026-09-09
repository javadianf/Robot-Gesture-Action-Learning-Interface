% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

[Names Path Ind] = uigetfile('.mat','Select','MultiSelect','on')
Full_path = 'E:\<PATH>\ski_2\';
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
for i=1:20:size(Names,2)
    load([Path Names{i}])
            if(i<10)
            zeros_ = '00';
            elseif(i>=10 && i<100)
              zeros_ = '0';
            elseif(i>=100)
              zeros_ = '';
            end
    save([Full_path 'image_' zeros_ num2str(i) '.mat'],'Om_IMG','Current_time');
end 
load([Path Names{size(Names,2)}])
save([Full_path 'image_' zeros_ num2str(size(Names,2)) '.mat'],'Om_IMG','Current_time');    
