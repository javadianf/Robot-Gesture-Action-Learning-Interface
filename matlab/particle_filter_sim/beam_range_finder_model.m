% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function NP = beam_range_finder_model(simstate)
%INITIALIZATION------------------------------------------------------------
K=size(simstate.robot.measurement,2);

M = simstate.particlefilter.num_particles;

Zhit=0.6;
Zrand=0.2;
Zmax=0.2;
Zshort=0;
Rmax = 2.5;
ahit=0.4;
%--------------------------------------------------------------------------
zst= simstate.robot.measurement(1,:);
for i=1:M;
    q(i,1)=1;
    %[zsta, rangebearing]  = ray_casting(simstate.particlefilter.particles.position(:,i),simstate);
    zt(1,:)=simstate.particlefilter.particles.measurement(1,:,i);
    for k=1:K
        %Zhit=zst(1,k);
        %Phit------------------------------------------------------------------
        Phit=0;
        if  0 <= zt(1,k) && zt(1,k) <= Rmax
            %Phit=(1/(sqrt(2*pi*ahit^2)))*exp(-0.5*(zt(1,k)-zst(1,k))^2/(ahit^2));
            Phit=(1/(sqrt(2*pi*ahit^2)))*exp(-0.5*(-zt(1,k)+zst(1,k))^2/(ahit^2));
        end
        %----------------------------------------------------------------------
        %Pmax------------------------------------------------------------------
        Pmax=0;
        if zt(1,k)==Rmax
            Pmax=1;
        end
        %----------------------------------------------------------------------
        %Prand-----------------------------------------------------------------
        Prand=0;
        if   0 <= zt(1,k) && zt(1,k) < Rmax
            Prand=1/Rmax;
        end
        %----------------------------------------------------------------------
        p=Zhit*Phit+Zmax*Pmax+Zrand*Prand;
        q(i,1) = q(i,1) * p;
    end
end
% NP=q';
Max=max(q);
Min=min(q);
NP=(q'-Min)./(Max-Min);
