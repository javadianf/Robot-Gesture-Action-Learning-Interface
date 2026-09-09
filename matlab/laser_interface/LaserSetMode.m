function sensor_Laser = sensorLaserSetMode( sensor_Laser, mode )
%sensorLaserReset
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser:-Object
%Output:    sensor_Laser:-Object
%           mode: Two modes are aviable: 'Operation' and 'Install'
%
%Description:   This function set the Operation state of the sensor-Scanner.
%               To make settinge, it must work in the Install-mode. For 
%               getting scan-Data the Operation-mode is need.
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de

switch mode
    case 'Operation'
        CMD = hex2dec('20');
        MODE = hex2dec('25');
        sensor_Laser.Status = 'Operation';
    case 'Install'
        sensor_Laser.Status = 'Install';
        CMD = hex2dec('20'); % Optimierbar
        MODE = 'sensor_LMS';%[0x53 0x49 0x43 0x4B 0x5F 0x4C 0x4D 0x53]; % password: "sensor_LMS", 
    otherwise
        disp('select a mode: Operation or Install');
        return;
end
[ sensor_Laser,msg ] = sensorLaserSendTelegram( sensor_Laser,CMD,MODE,10,0);

%% Check if ACK
if(length(msg)<10)
    disp('ERROR: No Data receive during sensorLaserSetMode().');
    return
end

if ((msg(1)==6)&&(msg(2)==2))
    % all right
else
    disp('ERROR: No Ack receive during sensorLaserSetMode().');
end
end

