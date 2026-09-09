function msg = getMsg(sensor_port,len)

delay(sensor_port,0);
if sensor_port.BytesAvailable
    msg = fread(sensor_port,len,'uint8');
else
    msg = [];
end

