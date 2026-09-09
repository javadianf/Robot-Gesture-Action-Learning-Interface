% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function distance = euclideanDistanceMatrix(tempPop,diagInput)
% DistCalc2 - calculates the distance between all objectives
% =========================================================================
%
%   distance = euclideanDistanceMatrix(tempPop,diagInput)
%
%   The function calculates the (euclidean) distance  between all 
%   objectives of the members of tempPop.
% 
%   Input:
%       "tempPop":  is a Matrix; every column correspons to one Individuum 
%                   of the Population.
%       "diagInput":stores the value who is written later on the diogonal 
%                   of the 
%       "distance": Matrix. (distance to itself)
% 
%   Output:
%       Output is a "distance" Matrix, describing the distances between all 
%       members of the Population. For example distance(1,2) stores the 
%       euclidean distance between Individuum 1 and Individuum 2. (The 
%       distance Matrix is symmetric.)
% 
% Jan Braun
% Lehrstuhl RST 
% TU x
% 27.06.2007
%

%% ========================================================================

popSize = size(tempPop,2); % number of columns (individuums)
distance = diag(ones(popSize,1)*diagInput); % filling the diaogonal and preallocation
for i = 1:popSize-1 % first Population counter
    Individuum = tempPop(:,i); % copy current values
    distVector = sqrt(sum((tempPop(:,i+1:end)-(Individuum*ones(1,popSize-i))).^2,1));
    distance(i,i+1:end) = distVector; % symmetric matix; results are copied into the row
    distance(i+1:end,i) = distVector; % symmetric matix; results are copied into the column
end
