function sensor_Laser = sensorLaserChgkBaudRate( sensor_Laser, baud_rate )
%sensorLaserChgkBaudRate
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser-Object
%           baud_rate: Baudrate in kBaud
%Output:    sensor_Laser-Object
%
%Description:   This function set the baud_rate for the sensor laser and the
%               PC-Port
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de
if(strcmp(sensor_Laser.Status,'Install')==0)
    disp('Laser must in Install-mode');
    return
end

switch baud_rate
    case 9600
        MODE = hex2dec('42');
    case 19200
        MODE = hex2dec('41');
    case 38400
        MODE = hex2dec('40');
    case 500000
        MODE = hex2dec('48');        
    otherwise
    disp('unknown baud_rate');
    return;
end
sensor_Laser.Baud_rate = baud_rate;
CMD = hex2dec('20');

[ sensor_Laser,~ ] = sensorLaserSendTelegram( sensor_Laser,CMD,MODE,0,0);
% Configure the PC-COM-Port
pause(1);
sensor_Laser.sensor_port.BaudRate = baud_rate;


fclose(sensor_Laser.sensor_port);
pause(0.1);
fopen(sensor_Laser.sensor_port);

% No Reply because of BaudRate-Change
% msg = getMsg(sensor_Laser.sensor_port,10);
% 
% a = 4;
end

