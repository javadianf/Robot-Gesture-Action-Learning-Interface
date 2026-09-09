function sendMsg(sensor_port,msg)

fwrite(sensor_port,msg,'uint8','async');
delay(sensor_port,0)
