% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

[Names Path Ind] = uigetfile('.mat','Select','MultiSelect','on')
Data_mat=zeros(size(Names,2),6);
for i=1:size(Names,2)
    load([Path Names{i}])
Data_mat(i,1)=tvel;
Data_mat(i,2)=rvel;
Data_mat(i,3:5)=Current_time;
Data_mat(i,6)=(Data_mat(i,5)-Data_mat(1,5))+60*(Data_mat(i,4)-Data_mat(1,4))+3600*(Data_mat(i,3)-Data_mat(1,3));
end 
TvelMean=mean(Data_mat(i,1))
RvelMean=mean(Data_mat(i,2))
Curveture = Data_mat(i,2)./ Data_mat(i,1);
Curv_Var=var(Curveture)
% figure(1)
% plot(Data_mat(i,6),Data_mat(i,1),'k',Data_mat(i,6),TvelMean)
% figure(2)
% plot(Data_mat(i,6),Data_mat(i,2),'k')
% figure(3)
% plot(Data_mat(i,6),Curveture,'k')
 
