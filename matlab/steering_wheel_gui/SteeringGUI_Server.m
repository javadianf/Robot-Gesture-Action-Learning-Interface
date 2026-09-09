% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function varargout = SteeringGUI_Server(varargin)
%�Steering Teleoperation GUI for driving with visual feedback
%�========================================================================
%
%��Description:
%����Creates a GUI to visualize the output parameters from a simulink file
%and camera vision from the robot's camera. Basic functions include
%interfaces to connect the the robot by calling aria interface commands,
%connect to another MATLAB instance with TCP/IP protocal, and stop the
%background Simulink file. Radio buttons select for the camera view the
%user wishes the see when driving the robot. Output parameters on the
%bottom of the GUI display the angle of the steering wheel, the ouput
%rotational velocity, the percent throttle of the gas pedal, and the output
%translational velocity.
%
%This is the server side of the system, that directly is plugged into the
%robot.
%
%Tested only with the MOMO force_feedback steering wheel.
%
%��Known Bugs:
%����There are times when the connection is lost between the server and the
%internet, which causes the server to stop responding to commands given by
%the client.
%
%��xs:
%����Winslow Cho
%
%
%�xxxt x 2009 RST, Technische Universit�t x, x
%���������������������www.rst.e-technik.tu-x.de
%

%%�=======================================================================

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @SteeringGUI_Server_OpeningFcn, ...
    'gui_OutputFcn',  @SteeringGUI_Server_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

% End initialization code - DO NOT EDIT


% --- Executes just before SteeringGUI_Server is made visible.
function SteeringGUI_Server_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SteeringGUI_Server (see VARARGIN)

% Choose default command line output for SteeringGUI_Server
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Initialize all global variables to be used in the GUI and obtain a handle
%for the robot omnidirectional camera. Also, create a Server socket for
%TCP/IP communication
global camera_on
global connectionServer
global max_rotvel;
global max_transvel; % mm/s
global rotvel;
global transvel;
global ocam_model
global c
global r
global rec
global image_par
global tcpipServer
global hnd_omni
global mobilecontrols
global collisionradius
global error
global sonarrange
global database
global Saving_on
global Save_CheckBox_ON
global Mat_Add

Save_CheckBox_ON = 0;
Saving_on = 0;
error = 0;
mobilecontrols = [0,0,1];
sonarrange = uint8([255;255;255;255;255;255;255;255]);
collisionradius = [0.5;0.6;0.9;1;1;0.9;0.6;0.5;0.3;0.2;0.1;0;0;0.1;0.2;0.3];
database = (1:0.2745:71).^2;
%Create the server socket (Open the connection from all IP addresses)
tcpipServer = tcpip('0.0.0.0',30000,'NetworkRole','Server');
set(tcpipServer,'OutputBufferSize',361235);
set(tcpipServer,'InputBufferSize',24);
set(tcpipServer,'Timeout',60);
max_rotvel = 75;
max_transvel = 300; % mm/s
rotvel = 0;
transvel = 0;
camera_on = 0;
connectionServer = 0;
%Load variables for the CV
load ocam_model;
ocam_model.xc       = 636;
ocam_model.yc       = 471;
c                   = [ocam_model.xc  ocam_model.yc];
r                   = 310;
rec                 = round([c(1)-r c(2)-r 2*r 2*r]); %rectangle inscribed in the square
image_par.xc= 628;
image_par.yc= 477;
image_par.large_radius= 441.8653;
image_par.small_radius= 75.1747;
image_par.above_radius= 340.9081;
image_par.rect= [186 35 884 884];
%Request handle for the omnidirectional camera
imaqreset;
hnd_omni = getHandleForDFx41AF02();
start(hnd_omni)

% UIWAIT makes SteeringGUI_Server wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SteeringGUI_Server_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in robotconnection.
function robotconnection_Callback(hObject, eventdata, handles)
% hObject    handle to robotconnection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Use aria interface to connect and disconnect from the robot
if strcmp(get(handles.robotconnection,'String'),'Start Robot')
    set(handles.robotconnection,'String','Disconnect Robot');
    robotInit();
else
    set(handles.robotconnection,'String','Start Robot');
    shutdown();
end

% --- Executes on button press in serverconnect.
function serverconnect_Callback(hObject, eventdata, handles)
% hObject    handle to serverconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Initialize all global variables to be used in the robot control
global connectionServer
global tcpipServer
global camera_on
global hnd_omni
global ocam_model
global c
global r
global rec
global image_par
global mobilecontrols
global max_transvel; % mm/s
global transvel;
global shownImage
global collisionradius
global error
global sonarrange
global database
global Saving_on
global Save_CheckBox_ON
global nCounter
global Full_path
nCounter = 0;
if strcmp(get(handles.serverconnect,'String'),'Open Server')
    set(handles.serverconnect,'String','Connecting Client');
    pause(0.005);
    %Open the server socket, the server will block all other commands until
    %a Client has successfully connected to the server
    fopen(tcpipServer);
    set(handles.serverconnect,'String','Server Running');
    connectionServer = 1;
    while connectionServer==1
        %If bytes available to read, then read information from Client
        if tcpipServer.bytesAvailable >= 24
            mobilecontrols = fread(tcpipServer,3,'double');
            set(handles.rotvel,'String',num2str(mobilecontrols(1)));
            set(handles.transvel,'String',num2str(mobilecontrols(2)));
            set(handles.cameraview,'String',num2str(mobilecontrols(3)));
            %Flush input buffer to prevent lagging
            flushinput(tcpipServer);
        end
        
        if strcmp(get(handles.robotconnection,'String'),'Disconnect Robot')
            % Collision detection vaiables from Sonar
            proximity_sensor = sonarSensorRange();
            %Convert the proximity information from the Sonar to shorter
            %uint8 accessible bytes
            sonarrange = proximity_sensor(1:8);
            
            for i=1:8
                [distance index] = min(abs(database-sonarrange(i)));
                sonarrange(i) = index;
            end
            %Determine distance for collision prevention
            collision = sum(proximity_sensor<(1000*collisionradius));
            emergency_stop = sum(proximity_sensor<(350*collisionradius));
            
            if collision >=1
                if emergency_stop >=1
                    transvel = 0;
                else
                    transvel = min(proximity_sensor)*mobilecontrols(2)*max_transvel/120000;
                end
            else
                transvel = mobilecontrols(2)*max_transvel/100;
            end
            
            set(handles.actrotvel,'String',num2str(mobilecontrols(1)));
            set(handles.acttransvel,'String',num2str(transvel));
            %Use aria interface to relay velocities to the robot
            setRotVel(mobilecontrols(1));
            setVel(transvel);
        end
        %Obtain image from the camera and process according to Client
        %request. Then attempt to send image.
        [Omni_Image_1] = get_Images(hnd_omni);
        switch mobilecontrols(3)
            case 1
                im = imcrop(Omni_Image_1,image_par.rect);
                Rows = size(im,1);
                Cols = size(im,2);
                [mapx mapy] = ocvBirdsEyeViewGetMap(100,346,Rows,Cols);
                [imBird]=ocvRemap(im,mapx,mapy);
                imBird = flipdim(imBird,2);
                imBird = imcrop(imBird,[100,100,693,693]);
                imBird = imresize(imBird,0.5);
                %                 imBird = imresize(imBird,0.392);
                flushoutput(tcpipServer);
                try
                    fwrite(tcpipServer,[reshape(imBird,361227,1);sonarrange],'uint8');
                catch
                    error = error+1;
                end
                shownImage = imBird;
            case 2
                OmniImage = imcrop(Omni_Image_1, [180 5, 930, 930]);
                OmniImage(:, :, 1) = fliplr(OmniImage(:, :, 1));
                OmniImage(:, :, 2) = fliplr(OmniImage(:, :, 2));
                OmniImage(:, :, 3) = fliplr(OmniImage(:, :, 3));
                OmniImage = imrotate(OmniImage,90);
                shownImage = uint8(unwarpSimple(OmniImage, 390, 600, 200));
                flushoutput(tcpipServer);
                try
                    fwrite(tcpipServer,[reshape(shownImage,360000,1);sonarrange],'uint8');
                catch
                    error = error+1;
                end
            case 3
                OmniImage = imcrop(Omni_Image_1, [180 5, 930, 930]);
                OmniImage(:, :, 1) = fliplr(OmniImage(:, :, 1));
                OmniImage(:, :, 2) = fliplr(OmniImage(:, :, 2));
                OmniImage(:, :, 3) = fliplr(OmniImage(:, :, 3));
                shownImage = imresize(OmniImage,0.372);
                flushoutput(tcpipServer);
                try
                    fwrite(tcpipServer,[reshape(shownImage,361227,1);sonarrange],'uint8');
                catch
                    error = error+1;
                end
            otherwise
                im = imcrop(Omni_Image_1,image_par.rect);
                Rows = size(im,1);
                Cols = size(im,2);
                [mapx mapy] = ocvBirdsEyeViewGetMap(100,346,Rows,Cols);
                [imBird]=ocvRemap(im,mapx,mapy);
                imBird = flipdim(imBird,2);
                imBird = imresize(imBird,0.392);
                flushoutput(tcpipServer);
                try
                    fwrite(tcpipServer,[reshape(imBird,361227,1);sonarrange],'uint8');
                catch
                    error = error+1;
                end
                shownImage = imBird;
        end
        %If too many errors occur, try to reestablish Server connection to
        %Client
        if error>5
            if strcmp(get(handles.robotconnection,'String'),'Disconnect Robot')
                setRotVel(0);
                setVel(0);
            end
            fclose(tcpipServer);
            error = 0;
            set(handles.serverconnect,'String','Connecting Client');
            pause(0.05);
            fopen(tcpipServer);
            set(handles.serverconnect,'String','Server Running');
        end
        
        %--------------------------------------------------
        % Saving all Data
        if (Saving_on == 1)
            nCounter = nCounter +1;      
            if(nCounter<10)
              zeros_ = '00';
            elseif(nCounter>=10 && nCounter<100)
              zeros_ = '0';
            elseif(nCounter>=100)
              zeros_ = '';
            end
            Data_Save(handles.rotvel, handles.transvel, handles.cameraview, sonarrange, Omni_Image_1,zeros_);
        end
        %--------------------------------------------------
        
        %If user asks to see images that are being sent, display images
        if camera_on==1
            handles.viewpreview = imshow(shownImage);
        end
        
        pause(0.033); %Neccessary for unlocking GUI
    end
else
    set(handles.serverconnect,'String','Open Server');
    %Close the server socket
    connectionServer = 0;
    fclose(tcpipServer);
end

% --- Executes on button press in camera.
function camera_Callback(hObject, eventdata, handles)
% hObject    handle to camera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Ask to see images that are being obtained and processed
% global camera_on

if strcmp(get(handles.camera,'String'),'View Camera Feed')
    set(handles.camera,'String','Close Camera Feed');
    camera_on = 1;
else
    set(handles.camera,'String','View Camera Feed');
    camera_on = 0;
end



%---------------------------------------------------------------
% --- Executes on button press in SaveChek.
function SaveChek_Callback(hObject, eventdata, handles)
% hObject    handle to SaveChek (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Save_CheckBox_ON

% Hint: get(hObject,'Value') returns toggle state of SaveChek
if (get(hObject,'Value') == get(hObject,'Max'))
    Save_CheckBox_ON = 1;
else
    Save_CheckBox_ON = 0;
end


% --- Executes on button press in Address
function Address_Callback(hObject, eventdata, handles)
% hObject    handle to Address (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on button press in StartDemo.
global Saving_Folder

Saving_Folder = uigetdir; % Saving folder address from user after click on Address buttom


% --- Executes on button press in StartDemo.
function StartDemo_Callback(hObject, eventdata, handles)
% hObject    handle to StartDemo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Saving_on % radiobutton
global Save_CheckBox_ON % push button
%global Saving_Folder % path name
global Basename % Address of .mat file containing saved data
Basename.Data = 'Demo_Data_';
Basename.Omni = 'Omni_Image_';
if (strcmp(get(handles.StartDemo,'String'),'Start demo'))
    if(Save_CheckBox_ON == 1)
        Saving_on = 1;
        set(handles.StartDemo,'String','Stop demo');
        tic;
    else
        Saving_on = 0;
    end
elseif(strcmp(get(handles.StartDemo,'String'),'Stop demo'))
    set(handles.StartDemo,'String','Start demo');
    Saving_on = 0;
end
    



% if strcmp(get(handles.StartDemo,'String'),'Save Data')
%     set(handles.StartDemo,'String','Stop Saving');
%     if Save_CheckBox_ON == 1 % only saves data if the save Chek box is ticked
%         Saving_on = 1;
%         Vel = zeros(1,3); % Building Vel matrix
%         Son = zeros(1,8); % Building Sonar matrix
%         Tag = datestr(now,0); % Building Tag matrix
%         DataSave = struct('Tag', Tag, 'Vel', Vel, 'Son', Son); % A structure containing all saved data
%         % Saving address
%         Mat_Add = strcat(Saving_Folder, '\', Mat_Name);
%         save(Mat_Add, 'DataSave')
%     else
%         Saving_on = 0;
%     end
% else
%     set(handles.StartDemo,'String','Save Data');
%     Saving_on = 0;
% end
%--------------------------------------------------------------
