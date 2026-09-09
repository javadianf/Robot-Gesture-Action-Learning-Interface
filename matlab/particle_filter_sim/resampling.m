% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function New_samples=resampling(simstate)
tresh=0.2;
Prob=simstate.particlefilter.particles.probability;
M=simstate.particlefilter.num_particles;
OrPa=zeros(3,M);
% [y,I]=sort(Prob,2,'ascend');
% 
% for i=1:M
%     OrPa(:,i)=simstate.particlefilter.particles.position(:,I(1,i));
% end

OrPa=simstate.particlefilter.particles.position;

% j=0;
% s=0;
% for i=round(M/2):M
%     s=s+Prob(1,i);
% end
% base=OrPa;
% i=0;
% while i<M
%     i=i+1;
%     dim=round(Prob(1,M-i+1)*M)/(2*s);
%     for j=0:dim-1
%         base(:,i+j)=OrPa(:,i);
%     end
% end
% New_samples=base;


% j=M;
w=0;
base=OrPa;
for i=1:M
    t=round(100*(Prob(1,i))^1);
%     if t<100*(tresh^5);
%         t=2;
%     end
    for k=1:t
        w=w+1;
        base(:,w)=OrPa(:,i);
    end
end
sz=size(base,2);
%r = randi([1,sz],1,M)
per=0.95;
r = round(1 + (sz-1).*rand(1,per*M));
New_samples=zeros(3,M);
for i=1:per*M
    New_samples(:,i)=base(:,r(i));
end


    for i =per*M+1:M
        New_samples(1,i)  = rand*simstate.enviroment.boundaryxr;
        New_samples(2,i)  = rand*simstate.enviroment.boundaryyu;
        New_samples(3,i)  = rand*2*pi;
        % at the start uniform distribution
        %simstate.particlefilter.particles.probability(i) = 1/M;
    end
