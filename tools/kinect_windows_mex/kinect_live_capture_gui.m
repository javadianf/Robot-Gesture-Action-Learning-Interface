% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.


%I need to assign a path to a new folder
% mkdir('Sample_Data',...  %parent directory
%       'Test1')             %new folder name

function kinect_live_capture_gui()
global DataMatrix;
global Previous_val;
global aviobj;
DataMatrix = [];
fig = figure;

%create the button
hButton = uicontrol(fig,'style','togglebutton');
set(hButton,'String','Start');
set(hButton,'FontSize',12,'ForegroundColor','g'); 
set(hButton,'Value',0);
Previous_val = 0;

while(1); 
    [a b c jx jy jz] = Kinect2(); 
    subplot(1,3,1); 
    imagesc(a); %depth map
    axis image; 
    colorbar;
    
    subplot(1,3,2); 
    imagesc(b); %player map
    axis image; 
    colorbar;
    playerIndex = max(max(b));
    
    if (playerIndex)
        %draw a circle on the head and both hands
        hold on;
        radius = 14;
        THETA=linspace(0,2*pi,6);
        RHO=ones(1,6)*radius;
        [X,Y] = pol2cart(THETA,RHO);
        HeadIndex = (playerIndex-1)*20+4; %HEAD is the 4th one
        X=X+jx(HeadIndex); 
        Y=Y+jy(HeadIndex);
        plot(X,Y,'-g','LineWidth',4);
        [X,Y] = pol2cart(THETA,RHO);
        LHandIndex = (playerIndex-1)*20+8; %Left Hand is the 8th one
        X=X+jx(LHandIndex); 
        Y=Y+jy(LHandIndex);
        plot(X,Y,'-g','LineWidth',4);
        [X,Y] = pol2cart(THETA,RHO);
        RHandIndex = (playerIndex-1)*20+12; %Right Hand is the 12th one
        X=X+jx(RHandIndex); 
        Y=Y+jy(RHandIndex);
        plot(X,Y,'-g','LineWidth',4);
        hold off;
    else  %this part is for later when we're gonna save the data
        HeadIndex  = 1;
        LHandIndex = 1;
        RHandIndex = 1;
    end
    
    subplot(1,3,3); 
    imagesc(c/255); % color map
    axis image; 
     drawnow; 

    % Create a Struct of the data 
    Head  = struct ('x', jx(HeadIndex),... % Assign x coordinates
                    'y', jy(HeadIndex),... % Assign y coordinates
                    'z', jz(HeadIndex));   % Assign z (depth)
    LHand = struct ('x', jx(LHandIndex),...
                    'y', jy(LHandIndex),...
                    'z', jz(LHandIndex));
    RHand = struct ('x', jx(RHandIndex),...
                    'y', jy(RHandIndex),...
                    'z', jz(RHandIndex));

    Data = struct('Depth'    ,a,...  
                  'Player'   ,b,...  
                  'Head'     ,Head,...   
                  'LeftHand' ,LHand,...   
                  'RightHand',RHand,...
                  'Label'    ,[]);      
                     
    %%%%%%%%%%%%%%for capturing data we have a toggle button for start and
    %%%%%%%%%%%%%%stop the process of capturing data
    
    %Check current state of the object
    val = get(hButton,'Value');
    if val==1 && Previous_val == 1      %actually capture data
        %first color map to AVI video
        F = im2frame(c/255);  % Convert "c" to a movie frame
        %aviobj = addframe(aviobj,F);  % Add the frame to the AVI file
        writeVideo(aviobj,F);

        %then deal with the data struct
        DataMatrix = [DataMatrix ; Data];
    end

    %calling the function in case the button was pushed
    set(hButton,'Callback',{@StartStopObject}) ;
           
end;



function StartStopObject(hButton, ~)
% Callback for the start/stop button
global cl;
global DataMatrix;
global Previous_val;
global aviobj;

%set path for saving files
Path = '<LOCAL_PATH>\';

val = get(hButton,'Value');

if    Previous_val == 0 && val == 1   % START just pushed, start capturing process
    set(hButton,'String','Stop','ForegroundColor','r');
    Previous_val = 1;
    
    cl = clock;
    AVIFilename = sprintf('Video-%d-%d-%d--%d-%d-%d.avi', cl(1:5),round(cl(6)));
    %create an AVI file to record stream
    %aviobj = avifile(AVIFilename,'fps',15); 
    aviobj = VideoWriter([Path AVIFilename]);
    aviobj.FrameRate = 10;
    open(aviobj);  

elseif Previous_val == 1 && val == 0   % STOP just pushed, finish capturing
    set(hButton,'String','Start','ForegroundColor','g');
    Previous_val = 0;
    matFilename = sprintf('Data--%d-%d-%d--%d-%d-%d.mat', cl(1:5),round(cl(6))); 
    %aviobj = close(aviobj);   % Close the AVI file
    close(aviobj);
    save ([Path matFilename], 'DataMatrix');  %save everything in a MAT file
    DataMatrix = [];    
    
end;

