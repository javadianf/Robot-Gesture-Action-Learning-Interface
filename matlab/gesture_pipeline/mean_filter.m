% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function dataFilt = mean_filter(data, size)
nLength = length(data);
dataTemp = data;
for i=1+floor(size/2):nLength-floor(size/2)
    dataTemp(i) = data(i-floor(size/2):i+floor(size/2))*ones([size 1])./size;
end
dataFilt = dataTemp;