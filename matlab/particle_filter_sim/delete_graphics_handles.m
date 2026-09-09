% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function delete_graphics_handles(last_r,last_rm,last_rrb,last_mr)

% handle to true robot postion
try
    delete(last_r);
catch
    disp('warning handle to true robot postion cannot be removed')
end

% handle to robot measurement
try
    delete(last_rm);
catch
    disp('warning handle to robot measurement cannot be removed')
end

% handle to robot' rangebearing measurements
try
    delete(last_rrb);
catch
    disp('warning handle to robot rangebearing measurements cannot be removed')
end

% handle to particle measurements
try
    for i =1:size(last_mr,2)
        delete(last_mr(i));
    end
catch
    disp('warning handle to to particle measurements cannot be removed')
end

    