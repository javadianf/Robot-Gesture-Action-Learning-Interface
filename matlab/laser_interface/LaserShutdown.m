function  sensorLaserShutdown( sensor_Laser )
%sensorLaserShutdown
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser:-Object
%
%Description:   This function shutdown the communication
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de
fclose(sensor_Laser.sensor_port ); 

end

