function [ sensor_Laser ] = sensorLaserInit( PORT, BAUDRATE )
%sensorLaserInit
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     PORT: Name of the Port (string)
%           BAUDRATE: in kBaud
%Output:    sensor_Laser:-Object
%
%Description:   This function starts the communication with the Laserscaner. 
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de


%! ->  maybee configure TimeOut, OutputBufferSize, OutputBufferSize for optimization purposes
sensor_Laser.sensor_port = serial(PORT,'BaudRate',BAUDRATE);
sensor_Laser.sensor_port.InputBufferSize = 2048;
sensor_Laser.Data_Len = 722; 
% Setzten das modus Aus Run
sensor_Laser.Baud_rate = BAUDRATE;
sensor_Laser.ScanMode  = 4;  % default
fopen(sensor_Laser.sensor_port);

sensor_Laser = sensorLaserSetMode(sensor_Laser,'Install');
sensor_Laser = sensorLaserChgScanMode(sensor_Laser,sensor_Laser.ScanMode );
sensor_Laser = sensorLaserSetMode(sensor_Laser,'Operation');

%-> Auslesen aus der Unit



end

