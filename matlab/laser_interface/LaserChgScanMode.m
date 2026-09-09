function sensor_Laser = sensorLaserChgScanMode( sensor_Laser, Mode )
%sensorLaserChgScanMode
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser-Object
%           Mode:   1 width: 100� and Step: 0.25�
%                   2 width: 100� and Step: 0.5�
%                   3 width: 100� and Step: 1.0�
%                   4 width: 180� and Step: 0.5�
%                   5 width: 180� and Step: 1.0�
%Output:    sensor_Laser-Object
%
%Description:   This function set the Scan Mode:(width and solution)
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de
if(strcmp(sensor_Laser.Status,'Install')==0)
    disp('Laser must in Install-mode');
    return
end

switch Mode
    case 1 % 100�  Step 0.25�
        CMD  = hex2dec('3B') ;        
        MODE = [100 0 25 0];
        sensor_Laser.Data_Len = 802; 
    case 2 % 100�  Step 0.5�
        CMD= hex2dec('3B');        
        MODE = [100 0 50 0];
        sensor_Laser.Data_Len = 402; 
    case 3 % 100�  Step 1.0�
        CMD = hex2dec('3B') ;        
        MODE = [100 0 100 0];
        sensor_Laser.Data_Len = 202; 
    case 4 % 180�  Step 0.5� % default After an Reset
        CMD = hex2dec('3B');        
        MODE = [ 180 0 50 0];
        sensor_Laser.Data_Len = 722; 
    case 5 % 180�  Step 1.0�
        CMD = hex2dec('3B')  ;        
        MODE = [180 0 100 0];
        sensor_Laser.Data_Len = 362; 
    otherwise
        disp('Wrong Mode in sensorLaserChgScanMode()')
        return;
end


[ sensor_Laser,msg ] = sensorLaserSendTelegram( sensor_Laser,CMD,MODE,14,1);
if(length(msg)<10)
    disp('ERROR: No Data receive during sensorLaserChgScanMode().');
    return
end
if(msg(6)~=hex2dec('BB'))
    disp('ERROR: No ACK received during sensorLaserChgScanMode().');
    return
end
if(msg(7)==hex2dec('00'))
    disp('ERROR: switchover aborted in sensorLaserChgScanMode().');
    return
end
end

