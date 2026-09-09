% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function [] = plot_trajectory_map(koord, class,classcolor)

if(nargin==1)%standard color for classes if none given
    class=ones(size(koord,1),1);
end

if(nargin<3)%standard color for classes if none given
    classcolor=[1,0,0;...%red
                0,1,0;...%green
                0,0,1];%blue
end

load('map.mat');
% index map
% map=rst_map-205;
% [mapy,mapx]=find(map~=0);
% map=[mapx-2000,-mapy+2000];
plot(map(:,1),map(:,2),'Color',[0.8 0.8 0.8],'Marker','.','MarkerSize',10,'LineStyle','none')


hold on
%�ndere farbe je nach klasse
    for i =1:3
%         if(~isempty(find(koord(i,:) == 0)))
            plot(koord((class==i),1)./0.05,koord((class==i),2)./0.05,'Color',classcolor(i,:),'Marker','.','LineStyle','none');
%         end
    end
hold off
axis equal

if(nargin~=1)
    legend('lasermap','corridor','open','cluttered')
end

end