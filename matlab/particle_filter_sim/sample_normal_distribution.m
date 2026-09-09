% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function s = sample_normal_distribution (b2)
b = sqrt(b2);
M =  b + (-b-b).*rand(1,2);
s =(sqrt(6)/2)*sum(M);