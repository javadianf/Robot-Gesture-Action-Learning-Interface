function sensor_Laser = sensorLaserChgMeasurementMode( sensor_Laser, Mode )
%sensorLaserChgMeasurementMode
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser-Object
%           Mode: Measurment in 'mm' or 'cm'
%Output:    sensor_Laser-Object
%
%Description:   This function set the MeasurementMode in 'mm' or 'cm'
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de


switch Mode
    case 'mm' % Messbereich 8 m/80 m; Reflektorbits in 8 Stufen
        CMD  = hex2dec('77')  ; 
%         MODE = [1 0];
%         MODE = [0 0 0  0 0 0 1 0 0 2 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2 0];
        MODE   = [0 0 70 0 0 0 1 0 0 2 2 2 0 0 10 10 80 100 0 10 10 80 128 00 10 10 80 100 0 0 0 0 2 0];


    case 'cm' % Messbereich 8 m/80 m; Feld A, Feld B und Feld C
        CMD  = hex2dec('77')  ; 
%         MODE = [2 0]  ;   
%         MODE = [0 0 0 0 0 0 0 0 0 2 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2 0];
        MODE   = [0 0 70 0 0 0 0 0 0 2 2 2 0 0 10 10 80 100 0 10 10 80 128 00 10 10 80 100 0 0 0 0 2 0];

    otherwise
        disp('Wrong MeasurementMode allowed are: "cm" or "mm"');
        return;
end

STX = 2;
ADR = 0;

msg_body = [CMD MODE];

% bulid telegramm
msg = [STX ADR LENL(length(msg_body)) ...
     LENH(length(msg_body)) msg_body];

% compute and add crc
crc16 = crc16alg(msg);
msg = [msg LENL(crc16) LENH(crc16)];
sendMsg(sensor_Laser.sensor_port,msg);

%% Check if ACK
pause(7); % wait 7 sec
pause(0.1);
msg = getMsg(sensor_Laser.sensor_port,9);
  a = 1;
  pause(0.1);
if(length(msg)<9)
    disp('ERROR: No Data receive during sensorLaserChgScanMode().');
    return
end
% if(msg(5)~=hex2dec('BB'))
%     disp('ERROR: No ACK received during sensorLaserChgScanMode().');
%     return
% end
if(msg(6)==hex2dec('00'))
    disp('ERROR: switchover aborted in sensorLaserChgScanMode().');
    return
end
sensor_Laser.MeasurementMode  = Mode;
end

