function [ sensor_Laser return_msg ] = sensorLaserSendTelegram( sensor_Laser,CMD,MODE,return_Msg_len,add_Pause)
%sensorLaserSendTelegram
%
%Autors:	Luis Felipe Posada, Oliver Birkholz
%e-mail:	felipe.posada@tu-dortmund.de, oliver.birkholz@tu-dortmund.de
%Date:      01.03.2012
%
%Input:     sensor_Laser:-Object
%           CMD: Part of the Message
%           MOD: Part of the Message
% return_Msg_len: spezifies the length of the return message. if this is 0,
%               This part will bee skiped
%      add_Pause: adds an additional wait time
%
%Output:    sensor_Laser:-Object
%           return_msg: Message from the sensor-scaner
%
%Description:   This function sends a telegramm to the sensor-scaner. if return_Msg_len is greater than 0 it will return the returnmessage from the Sensor.
%               Important: If a response is expected, this must be picked
%               up.
%
%Copyright (c) 2012 RST, Technische Universit�t Dortmund, Germany
%www.rst.e-technik.uni-dortmund.de

STX = 2;
ADR = 0;

msg_body = uint8([CMD MODE]);

% bulid telegramm
msg_send = [STX ADR LENL(length(msg_body)) ...
     LENH(length(msg_body)) msg_body];

% compute and add crc
crc16 = crc16alg(msg_send);
msg_send = [msg_send LENL(crc16) LENH(crc16)];
sendMsg(sensor_Laser.sensor_port,msg_send);

if(return_Msg_len>0)
    pause(0.1);
    pause(add_Pause);
    return_msg = getMsg(sensor_Laser.sensor_port,return_Msg_len);
else
    return_msg = [];
end
end

