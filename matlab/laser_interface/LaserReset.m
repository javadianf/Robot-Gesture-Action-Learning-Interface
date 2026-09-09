function  sensor_Laser = sensorLaserReset( sensor_Laser )
%sensorLaserReset
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser:-Object
%Output:    sensor_Laser:-Object
%
%Description:   This function makes a Hardware reset on the scaner
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de

CMD = hex2dec('10');

[ sensor_Laser,~ ] = sensorLaserSendTelegram( sensor_Laser,CMD,[],0,0);
sensor_Laser.baud_rate = 9600;
sensor_Laser.Stauts = 'Run';

end

