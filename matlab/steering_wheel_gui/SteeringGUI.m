% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function varargout = SteeringGUI(varargin)
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
%Tested only with the MOMO force_feedback steering wheel. 
% 
%��Known Bugs:
%����At times Matlab freezes and closes before the code initiates. Test to 
%make sure that the Simulink file can be complied separately before running 
%the GUI. 
%
%��xs:
%����Winslow Cho
%
%
%�xxxt x 2009 RST, Technische Universit�t x, x
%���������������������www.rst.e-technik.tu-x.de
%

%%�=======================================================================

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SteeringGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SteeringGUI_OutputFcn, ...
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


% --- Executes just before SteeringGUI_user is made visible.
function SteeringGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SteeringGUI (see VARARGIN)

% Choose default command line output for SteeringGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global smf;
global view1cue
global view2cue
global view3cue
global robotControlWheel
global flag

flag = 0;
smf = 0;
view1cue = 1;
view2cue = 0;
view3cue = 0;
robotControlWheel = [0,0];

% Opens and starts the simulink file to capture outputs from the steering wheel and
% pedals. Waits until the simulink file starts to run before continuing. 
% run Steering_Wheel_Timer;
% set_param('Steering_Wheel_Timer','SimulationCommand','start')
% while strcmp(get_param('Steering_Wheel_Timer','SimulationStatus'),'stopped')
% end
%t = 1; % Uncomment Code if you want to log data over time. 

% UIWAIT makes SteeringGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SteeringGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global max_rotvel;
global max_transvel; % mm/s
global rotvel;
global transvel;

% Sets predetermined values for maximum rotational and translational
% velocity as well as the intial values for these two velocities. 
max_rotvel = 75;
max_transvel = 300; % mm/s
rotvel = 0;
transvel = 0;
%t = 1; % Uncomment Code if you want to log data over time. 


% --- Executes when selected object is changed in Viewspanel.
function Viewspanel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in Viewspanel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
global view1cue
global view2cue
global view3cue

switch get(eventdata.NewValue,'Tag')
    case 'view1'
        view1cue = 1;
        view2cue = 0;
        view3cue = 0;
%         handles.cameraview = imshow(unwarped);
    case 'view2'
        view1cue = 0;
        view2cue = 1;
        view3cue = 0;
%         handles.cameraview = imshow(rgb2gray(unwarped));
    case 'view3'
        view1cue = 0;
        view2cue = 0;
        view3cue = 1;
%         temp = unwarped;
%         temp(:,:,2:3) = 0;
%         handles.cameraview = imshow(temp);
    otherwise
        view1cue = 1;
        view2cue = 0;
        view3cue = 0;
end


% --- Executes on button press in view1.
function view1_Callback(hObject, eventdata, handles)
% hObject    handle to view1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on button press in view2.
function view2_Callback(hObject, eventdata, handles)
% hObject    handle to view2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in view3.
function view3_Callback(hObject, eventdata, handles)
% hObject    handle to view3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in aria_init.
function aria_init_Callback(hObject, eventdata, handles)
% hObject    handle to aria_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if strcmp(get(handles.aria_init,'String'),'Robot Connect')
    % Robot is not connected to the program
    set(handles.aria_init,'String','Robot Disconnect');
    robotInit();
else
    set(handles.aria_init,'String','Robot Connect');
    shutdown();
end


% --- Executes on button press in tcpip_connection.
function tcpip_connection_Callback(hObject, eventdata, handles)
% hObject    handle to tcpip_connection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global tcpipClient;
global flag
global view1cue
global view2cue
global view3cue
global tempdata
global smf

if strcmp(get(handles.tcpip_connection,'String'),'TCP/IP')
    set(handles.tcpip_connection,'String','Disable Com');
    tcpipClient=tcpip('129.217.188.22',30000,'NetworkRole','Client'); % Use '127.0.0.1',55000 to communicate within the same computer
    set(tcpipClient,'InputBufferSize',360000);
    set(tcpipClient,'OutputBufferSize',16);
    set(tcpipClient,'Timeout',30);
    try
        fopen(tcpipClient);
        flag = 1;
    catch error
        set(handles.tcpip_connection,'String','TCP/IP');
        flag = 0;
    end
    if flag==1 && smf ==0
        while strcmp(get(handles.tcpip_connection,'String'),'Disable Com') && smf ==0
            if tcpipClient.bytesAvailable >= 360000
                data = fread(tcpipClient,360000,'uint8');
                tempdata = uint8(reshape(data,200,600,3));
                if view1cue==1
                    handles.cameraview = imshow(tempdata);
                elseif view2cue==1
                    handles.cameraview = imshow(rgb2gray(tempdata));
                elseif view3cue==1
                    temp = tempdata;
                    temp(:,:,2:3) = 0;
                    handles.cameraview = imshow(temp);
                else
                    handles.cameraview = imshow(tempdata);
                end
            end
            pause(0.005);
        end
    end
else
    set(handles.tcpip_connection,'String','TCP/IP');
    flag = 0;
    fclose(tcpipClient);
end


% --- Executes on button press in simulink_button.
function simulink_button_Callback(hObject, eventdata, handles)
% hObject    handle to simulink_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global smf;
global max_rotvel;
global max_transvel; % mm/s
global rotvel;
global transvel;
global robotControlWheel
global flag
global view1cue
global view2cue
global view3cue
global tempdata
global tcpipClient;

if strcmp(get(handles.simulink_button,'String'),'Simulink Stop')
    set(handles.simulink_button,'String','Simulink Start');
    set_param('Steering_Wheel_Timer','SimulationCommand','stop')
    smf=0;
else
    set(handles.simulink_button,'String','Simulink Stop');
    open_system('Steering_Wheel_Timer');
    set_param('Steering_Wheel_Timer','SimulationCommand','start')
    while 'stopped' == get_param('Steering_Wheel_Timer','SimulationStatus')
    end
    smf=1;
    % Opens and starts the simulink file to capture outputs from the steering wheel and
    % pedals. Waits until the simulink file starts to run before continuing. 
    while smf==1 && flag==0
        % Perpetually gathers information from Simulink and displays in the GUI

        % Grabs handle from Simulink to log output steering wheel variables. 
        rto = get_param('Steering_Wheel_Timer/Joystick2Micardo','RuntimeObject');

        if exist('rto')==1
        %steering_time(t) = rto.OutputPort(1).Data; % Uncomment Code if you want to log data over time. 
        steering_angle = rto.OutputPort(1).Data;
        set(handles.steerangle_text,'String',num2str(steering_angle)); % Handle for displaying steering angle

        %gas_time(t) = rto.OutputPort(2).Data; % Uncomment Code if you want to log data over time. 
        gas_percent = rto.OutputPort(2).Data;

        %brake_time(t) = rto.OutputPort(3).Data; % Uncomment Code if you want to log data over time. 
        brake_percent = rto.OutputPort(3).Data;

        % Brake input formula
        gas_brake = gas_percent-brake_percent;
        if gas_brake<0
            gas_brake = 0;
        end

        % Sets rotational and translational velocities based on steering angle
        % and pedal press percentage
        set(handles.gasbrake_text,'String',num2str(gas_brake));
        rotvel = -steering_angle*max_rotvel/115;
        set(handles.rotvel_text,'String',num2str(rotvel));
        
        transvel = gas_brake*max_transvel/100;
        set(handles.transvel_text,'String',num2str(transvel));
        end
        pause(0.005);
    end
    while smf==1 && flag==1
        % Perpetually gathers information from Simulink and displays in the GUI
        
        if tcpipClient.bytesAvailable >= 360000
            data = fread(tcpipClient,360000,'uint8');
            tempdata = uint8(reshape(data,200,600,3));
            if view1cue==1
                handles.cameraview = imshow(tempdata);
            elseif view2cue==1
                handles.cameraview = imshow(rgb2gray(tempdata));
            elseif view3cue==1
                temp = tempdata;
                temp(:,:,2:3) = 0;
                handles.cameraview = imshow(temp);
            else
                handles.cameraview = imshow(tempdata);
            end
%             fwrite(tcpipClient,robotControlWheel,'double');
        end

        % Grabs handle from Simulink to log output steering wheel variables. 
        rto = get_param('Steering_Wheel_Timer/Joystick2Micardo','RuntimeObject');

        if exist('rto')==1
        %steering_time(t) = rto.OutputPort(1).Data; % Uncomment Code if you want to log data over time. 
        steering_angle = rto.OutputPort(1).Data;
        set(handles.steerangle_text,'String',num2str(steering_angle)); % Handle for displaying steering angle

        %gas_time(t) = rto.OutputPort(2).Data; % Uncomment Code if you want to log data over time. 
        gas_percent = rto.OutputPort(2).Data;

        %brake_time(t) = rto.OutputPort(3).Data; % Uncomment Code if you want to log data over time. 
        brake_percent = rto.OutputPort(3).Data;

        % Brake input formula
        gas_brake = gas_percent-brake_percent;
        if gas_brake<0
            gas_brake = 0;
        end

        % Sets rotational and translational velocities based on steering angle
        % and pedal press percentage
        set(handles.gasbrake_text,'String',num2str(gas_brake));
        rotvel = -steering_angle*max_rotvel/115;
        set(handles.rotvel_text,'String',num2str(rotvel));
        
        transvel = gas_brake*max_transvel/100;
        
        robotControlWheel(1) = rotvel;   % Remove when controlling robot
        robotControlWheel(2) = transvel; % Remove when controlling robot

        % If robot is connected, write commands to aria interface
        if strcmp(get(handles.aria_init,'String'),'Robot Disconnect')
            % Collision detection vaiables from Sonar
            proximity_sensor = sonarSensorRange();
            collision = sum(proximity_sensor(3:6)<800);
            emergency_stop = sum(proximity_sensor(3:6)<300);

            if collision >=1
                if emergency_stop >=1
                    transvel = 0;
                else
                    transvel = min(proximity_sensor)*gas_brake*max_transvel/120000;
                end
            else
                transvel = gas_brake*max_transvel/100;
            end
            set(handles.transvel_text,'String',num2str(transvel));
            setRotVel(rotvel);
            setVel(transvel);
            
            robotControlWheel(1) = rotvel;
            robotControlWheel(2) = transvel;
        end

        %t=t+1; % Uncomment Code if you want to log data over time. 
        fwrite(tcpipClient,robotControlWheel,'double');
        robotControlWheel % To display data being sent
        pause(0.15);

        % Get default command line output from handles structure
        varargout{1} = handles.output;
        end
%         pause(0.005);
    end
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);

