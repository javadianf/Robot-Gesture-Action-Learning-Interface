% SPDX-License-Identifier: CC-BY-NC-ND-4.0
% Copyright (c) Javadian. All rights reserved.

function varargout = SteeringGUI_Client(varargin)
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
%This is the client side of the GUI.
%
%Tested only with the MOMO force_feedback steering wheel. 
% 
%��Known Bugs:
%����At times, the server may disconnect from the internet and no longer
%respond to the commands given by the client.
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
                   'gui_OpeningFcn', @SteeringGUI_Client_OpeningFcn, ...
                   'gui_OutputFcn',  @SteeringGUI_Client_OutputFcn, ...
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
function SteeringGUI_Client_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SteeringGUI_Client (see VARARGIN)

% Choose default command line output for SteeringGUI_Client
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Initialize global variables to used in multiple cells
global smf;
global viewcue
global robotControlWheel
global flag
global error
global sonarrange
global database
global sonarupdate
global sonarwarning
global sonarrangevalue

sonarupdate = 0;
sonarrange = [255;255;255;255;255;255;255;255];
sonarrangevalue = [4000;4000;4000;4000;4000;4000;4000;4000];
sonarwarning = [0,0,0,0,0,0,0,0];
database = (1:0.2745:71).^2;
error = 0;
flag = 0;
smf = 0;
viewcue = 1;
robotControlWheel = [0,0,1];

%Initializes the sonar view plot
area(handles.sonarview,[-1,0,1],[2,5,2],'basevalue',2);
hold on;
plot(handles.sonarview,[-9,-1],[2,2],'g-');
plot(handles.sonarview,[-8,-0.7],[8,3],'g-');
plot(handles.sonarview,[-6,-0.45],[13,3.5],'g-');
plot(handles.sonarview,[-2,-0.3],[18,4.3],'g-');
plot(handles.sonarview,[0.3,2],[4.3,18],'g-');
plot(handles.sonarview,[0.45,6],[3.5,13],'g-');
plot(handles.sonarview,[0.7,8],[3,8],'g-');
plot(handles.sonarview,[1,9],[2,2],'g-');
hold off;
set(handles.sonarview,'ytick',[]);
set(handles.sonarview,'xtick',[]);
set(handles.sonarview,'xlim',[-10,10]);
set(handles.sonarview,'ylim',[0,20]);

% UIWAIT makes SteeringGUI_Client wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SteeringGUI_Client_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Initialize variables to be used in controlling robot through MobileSim
global max_rotvel;
global max_transvel; % mm/s
global rotvel;
global transvel;

% Sets predetermined values for maximum rotational and translational
% velocity as well as the intial values for these two velocities. 
max_rotvel = 10;
max_transvel = 40; % mm/s
rotvel = 0;
transvel = 0;


% --- Executes when selected object is changed in Viewspanel.
function Viewspanel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in Viewspanel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

%Initialize variables to be used in determining which double to send to ask
%for the correct view from the server
global viewcue
global robotControlWheel

%Determines which radio button is selected and changes the double
%requesting a particular view 
switch get(eventdata.NewValue,'Tag')
    case 'view1'
        viewcue = 1;
        robotControlWheel(3) = 1;
    case 'view2'
        viewcue = 2;
        robotControlWheel(3) = 2;
    case 'view3'
        viewcue = 3;
        robotControlWheel(3) = 3;
    otherwise
        viewcue = 1;
        robotControlWheel(3) = 1;
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

%Luanches aria interface commands to link to MobileSim
if strcmp(get(handles.aria_init,'String'),'Sim Connect')
    % Robot is not connected to the program
    set(handles.aria_init,'String','Sim Disconnect');
    robotInit();
else
    set(handles.aria_init,'String','Sim Connect');
    shutdown();
end


% --- Executes on button press in tcpip_connection.
function tcpip_connection_Callback(hObject, eventdata, handles)
% hObject    handle to tcpip_connection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Initializes global variables to be used in referencing TCP/IP
%communication
global tcpipClient;
global flag
global viewcue
global tempdata
global smf

if strcmp(get(handles.tcpip_connection,'String'),'TCP/IP')
    set(handles.tcpip_connection,'String','Disable Com');
    %Creates a client socket to connect to the specified server socket
    tcpipClient=tcpip('129.217.188.22',30000,'NetworkRole','Client');
    set(tcpipClient,'InputBufferSize',361235);
    set(tcpipClient,'OutputBufferSize',24);
    set(tcpipClient,'Timeout',30);
    %Tests to see if the client can successuflly connect to the server
    try
        fopen(tcpipClient);
        flag = 1;
    catch error
        set(handles.tcpip_connection,'String','TCP/IP');
        flag = 0;
    end
    %Starts to read image data sent by the server and displays them on the
    %main axes
    if flag==1 && smf ==0
        while strcmp(get(handles.tcpip_connection,'String'),'Disable Com') && smf ==0
            if tcpipClient.bytesAvailable >= 360008 && viewcue==2
                data = fread(tcpipClient,360008,'uint8');
                tempdata = uint8(reshape(data(1:360000),200,600,3));
                imshow(tempdata,'Parent',handles.cameraview);
            elseif tcpipClient.bytesAvailable >= 361235
                if viewcue==1
                    data = fread(tcpipClient,361235,'uint8');
                    tempdata = uint8(reshape(data(1:361227),347,347,3));
                    imshow(tempdata,'Parent',handles.cameraview);
                elseif viewcue==3
                    data = fread(tcpipClient,361235,'uint8');
                    tempdata = uint8(reshape(data(1:361227),347,347,3));
                    imshow(tempdata,'Parent',handles.cameraview);
                else
                    data = fread(tcpipClient,361235,'uint8');
                    tempdata = uint8(reshape(data(1:361227),347,347,3));
                    imshow(tempdata,'Parent',handles.cameraview);
                    flushinput(tcpipClient);
                end
            end
            pause(0.005); %Necessary to unlock the GUI
        end
    end
else
    set(handles.tcpip_connection,'String','TCP/IP');
    %Breaks from image acquistion loop and closes the client socket
    flag = 0;
    fclose(tcpipClient);
end


% --- Executes on button press in simulink_button.
function simulink_button_Callback(hObject, eventdata, handles)
% hObject    handle to simulink_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Initialize all global variables to be used in this section
global smf;
global max_rotvel;
global max_transvel; % mm/s
global rotvel;
global transvel;
global robotControlWheel
global flag
global viewcue
global tempdata
global tcpipClient;
global error
global reconnect
global sonarrange
global database
global sonarupdate
global sonarwarning
global sonarrangevalue

if strcmp(get(handles.simulink_button,'String'),'Simulink Stop')
    set(handles.simulink_button,'String','Simulink Start');
    set_param('Steering_Wheel_Timer','SimulationCommand','stop')
    smf=0;
else
    set(handles.simulink_button,'String','Simulink Stop');
    % Opens and starts the simulink file to capture outputs from the steering wheel and
    % pedals. Waits until the simulink file starts to run before continuing. 
    open_system('Steering_Wheel_Timer');
    set_param('Steering_Wheel_Timer','SimulationCommand','start')
    while 'stopped' == get_param('Steering_Wheel_Timer','SimulationStatus')
    end
    smf=1;
    
    while smf==1 && flag==0
        % Perpetually gathers information from Simulink and displays in the
        % GUI when Client socket is closed

        % Grabs handle from Simulink to log output steering wheel variables. 
        rto = get_param('Steering_Wheel_Timer/Joystick2Micardo','RuntimeObject');

        if exist('rto')==1
        steering_angle = rto.OutputPort(1).Data;
        set(handles.steerangle_text,'String',num2str(steering_angle)); % Handle for displaying steering angle
        gas_percent = rto.OutputPort(2).Data;
        brake_percent = rto.OutputPort(3).Data;

        % GasBrake input formula
        gas_brake = gas_percent-brake_percent;
        if gas_brake<0
            gas_brake = 0;
        end

        % Sets rotational and translational velocities based on steering angle
        % and pedal press percentage
        set(handles.gasbrake_text,'String',num2str(gas_brake));
        if abs(-steering_angle) <= 5
            rotvel = 0;
        elseif abs(-steering_angle) >= 60
            rotvel = sign(steering_angle)*-1*max_rotvel;
        else
            rotvel = -steering_angle*max_rotvel/60;
        end
        set(handles.rotvel_text,'String',num2str(rotvel));
        
        transvel = gas_brake*max_transvel/100;
        set(handles.transvel_text,'String',num2str(transvel));
        end
        pause(0.005);
    end
    
    while smf==1 && flag==1
        % Perpetually gathers information from Simulink and displays in the
        % GUI when Client socket is open
        
        %Reads imcoming information coming from the Server and displays
        %image data
        if tcpipClient.bytesAvailable >= 360008 && viewcue==2
            data = fread(tcpipClient,360008,'uint8');
            tempdata = uint8(reshape(data(1:360000),200,600,3));
            sonarrange = uint8(data(360001:360008));
            imshow(tempdata,'Parent',handles.cameraview);
            sonarupdate = 1;
        elseif tcpipClient.bytesAvailable >= 361235
            if viewcue==1
                data = fread(tcpipClient,361235,'uint8');
                tempdata = uint8(reshape(data(1:361227),347,347,3));
                sonarrange = uint8(data(361228:361235));
                imshow(tempdata,'Parent',handles.cameraview);
            elseif viewcue==3
                data = fread(tcpipClient,361235,'uint8');
                tempdata = uint8(reshape(data(1:361227),347,347,3));
                sonarrange = uint8(data(361228:361235));
                imshow(tempdata,'Parent',handles.cameraview);
            else
                data = fread(tcpipClient,361235,'uint8');
                tempdata = uint8(reshape(data(1:361227),347,347,3));
                sonarrange = uint8(data(361228:361235));
                imshow(tempdata,'Parent',handles.cameraview);
                flushinput(tcpipClient);
            end
            sonarupdate = 1;
        end
        
        %Updates the sonar screens with information sent by the server
        if sonarupdate==1
            for i = 1:8
                sonarrangevalue(i) = database(sonarrange(i));
                if sonarrangevalue(i) < 600
                    sonarwarning(i) = 1;
                else
                    sonarwarning(i) = 0;
                end
            end
            set(handles.CL,'String',num2str(sonarrangevalue(4)));
            set(handles.CCL,'String',num2str(sonarrangevalue(3)));
            set(handles.CLL,'String',num2str(sonarrangevalue(2)));
            set(handles.L,'String',num2str(sonarrangevalue(1)));
            set(handles.CR,'String',num2str(sonarrangevalue(5)));
            set(handles.CCR,'String',num2str(sonarrangevalue(6)));
            set(handles.CRR,'String',num2str(sonarrangevalue(7)));
            set(handles.R,'String',num2str(sonarrangevalue(8)));
            
            x1 = -9*sonarrangevalue(4)/1500;
            x2 = -8*sonarrangevalue(3)/1500;
            x3 = -6*sonarrangevalue(2)/1500;
            x4 = -2*sonarrangevalue(1)/1500;
            x5 = 2*sonarrangevalue(5)/1500;
            x6 = 6*sonarrangevalue(6)/1500;
            x7 = 8*sonarrangevalue(7)/1500;
            x8 = 9*sonarrangevalue(8)/1500;
            y1 = 2;
            y2 = -0.6849*x2+2.5205;
            y3 = -1.7117*x3+2.7297;
            y4 = -8.0588*x4+1.8824;
            y5 = 8.0588*x5+1.8824;
            y6 = 1.7117*x6+2.7297;
            y7 = 0.6849*x7+2.5205;
            y8 = 2;
            
            area(handles.sonarview,[-1,0,1],[2,5,2],'basevalue',2);
            hold on;
            if sonarwarning(1)==0
                plot(handles.sonarview,[x1,-1],[y1,2],'g-');
            else
                plot(handles.sonarview,[x1,-1],[y1,2],'r-');
            end
            if sonarwarning(2)==0
                plot(handles.sonarview,[x2,-0.7],[y2,3],'g-');
            else
                plot(handles.sonarview,[x2,-0.7],[y2,3],'r-');
            end
            if sonarwarning(3)==0
                plot(handles.sonarview,[x3,-0.45],[y3,3.5],'g-');
            else
                plot(handles.sonarview,[x3,-0.45],[y3,3.5],'r-');
            end
            if sonarwarning(4)==0
                plot(handles.sonarview,[x4,-0.3],[y4,4.3],'g-');
            else
                plot(handles.sonarview,[x4,-0.3],[y4,4.3],'r-');
            end
            if sonarwarning(5)==0
                plot(handles.sonarview,[0.3,x5],[4.3,y5],'g-');
            else
                plot(handles.sonarview,[0.3,x5],[4.3,y5],'r-');
            end
            if sonarwarning(6)==0
                plot(handles.sonarview,[0.45,x6],[3.5,y6],'g-');
            else
                plot(handles.sonarview,[0.45,x6],[3.5,y6],'r-');
            end
            if sonarwarning(7)==0
                plot(handles.sonarview,[0.7,x7],[3,y7],'g-');
            else
                plot(handles.sonarview,[0.7,x7],[3,y7],'r-');
            end
            if sonarwarning(8)==0
                plot(handles.sonarview,[1,x8],[2,y8],'g-');
            else
                plot(handles.sonarview,[1,x8],[2,y8],'r-');
            end
            hold off;
            set(handles.sonarview,'ytick',[]);
            set(handles.sonarview,'xtick',[]);
            set(handles.sonarview,'xlim',[-10,10]);
            set(handles.sonarview,'ylim',[0,20]);
            
            drawnow;
            sonarupdate = 0;
        end

        % Grabs handle from Simulink to log output steering wheel variables. 
        rto = get_param('Steering_Wheel_Timer/Joystick2Micardo','RuntimeObject');

        if exist('rto')==1
            steering_angle = rto.OutputPort(1).Data;
            set(handles.steerangle_text,'String',num2str(steering_angle)); % Handle for displaying steering angle
            gas_percent = rto.OutputPort(2).Data;
            brake_percent = rto.OutputPort(3).Data;

            % Brake input formula
            gas_brake = gas_percent-brake_percent;
            if gas_brake<0
                gas_brake = 0;
            end

            % Sets rotational and translational velocities based on steering angle
            % and pedal press percentage
            set(handles.gasbrake_text,'String',num2str(gas_brake));
            if abs(-steering_angle) <= 5
                rotvel = 0;
            elseif abs(-steering_angle) >= 60
                rotvel = sign(steering_angle)*-1*max_rotvel;
            else
                rotvel = -steering_angle*max_rotvel/60;
            end
            set(handles.rotvel_text,'String',num2str(rotvel));

            transvel = gas_brake*max_transvel/100;
            set(handles.transvel_text,'String',num2str(transvel));

            robotControlWheel(1) = rotvel;
            robotControlWheel(2) = transvel;

            % If robot is connected to MobileSim, write commands to aria interface
            if strcmp(get(handles.aria_init,'String'),'Sim Disconnect')
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
            
            %Clear the output buffer sending information about robot
            %movement
            flushoutput(tcpipClient);
            %Try to write the mobile movement and view information to the
            %Server. If fail, then count error.
            try 
                fwrite(tcpipClient,robotControlWheel(:),'double');
            catch
                error = error+1
            end
            %If errors occur then try to reconnect the client to the server
            if error>5
                fclose(tcpipClient);
                set(handles.tcpip_connection,'String','TCP/IP');
                pause(0.05);
                try
                    fopen(tcpipClient);
                    set(handles.tcpip_connection,'String','Disable Com');
                    reconnect = 1;
                catch
                    set(handles.tcpip_connection,'String','TCP/IP');
                    reconnect = 0;
                end
                if reconnect==1
                    error=0;
                end
            end
           
            pause(0.15);

            % Get default command line output from handles structure
            varargout{1} = handles.output;
        end
    end
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
global tcpipClient;

% shutdown();
% fclose(tcpipClient);
% close_system('Steering_Wheel_Timer');
delete(hObject);
