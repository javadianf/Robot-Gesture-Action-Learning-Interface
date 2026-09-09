% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

clear all;
close all;
reply = input('Please enter the file name(TXT): ', 's');
a_mat = textread(reply);
time_bias=a_mat(1,7);
a_mat(:,7)=a_mat(:,7)-time_bias;
q=zeros(size(a_mat,1),4);
q(:,1)=a_mat(:,4); % omega
q(:,4)=a_mat(:,3); % z
R=zeros(size(a_mat,1),4);
[R(:,1) R(:,2) R(:,3)] = quat2angle(q);
R(:,4)=R(:,1)*(180/pi);