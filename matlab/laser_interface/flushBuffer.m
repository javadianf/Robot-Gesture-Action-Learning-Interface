function flushBuffer(sensor_port) 

delay(sensor_port,0);
if sensor_port.BytesAvailable
    fread(sensor_port,sensor_port.BytesAvailable,'uint8');
end

