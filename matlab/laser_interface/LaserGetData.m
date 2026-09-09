function [ msg_out sensor_Laser] = sensorLaserGetData( sensor_Laser )
%sensorLaserGetData
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser:-Object
%Output:    sensor_Laser:-Object
%           msg_Out: Outputdata from the Laserscaner
%
%Description:   This function get the Laserdata from the sensorscaner. 
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de


STX = 2;
ADR = 0;

CMD = hex2dec('30');
MODE = hex2dec('01'); 
msg_body = [CMD MODE]; 
% bulid telegramm
msg = [STX ADR LENL(length(msg_body)) ...
     LENH(length(msg_body)) msg_body];

% compute and add crc
crc16 = crc16alg(msg);
msg = [msg LENL(crc16) LENH(crc16)];
sendMsg(sensor_Laser.sensor_port,msg);
% get Message
% Check min-len
pause(0.1); % Wait 100ms
len_head = 11;
msg = getMsg(sensor_Laser.sensor_port,sensor_Laser.Data_Len+len_head);
if((length(msg)<len_head))
    % too short message
      msg_out = [];
      return;
end

STX_get = msg(1);
LMI_get = msg(2);
LENG_L_get = msg(3);
LENG_H_get = msg(4);
CMD_get = msg(5);
DATALENL_get = msg(6);
DATALENH_get = msg(7);

STATUS_get = msg(end-2);
CRC_L_get = msg(end-1);
CRC_H_get = msg(end);

sensor_Laser.Status = STATUS_get;
msg = msg(9:end-3);
msg_out = zeros(length(msg)/2,1);

for i = 0 :length(msg)/2-1
   msg_out(i+1) =  msg(i*2+1) + bitand(msg(i*2+2),31)*256; % with & 31 mask only the important data
end
end

