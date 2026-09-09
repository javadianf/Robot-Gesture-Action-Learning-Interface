function delay(sensor_port,time)
tic
while (strcmp(sensor_port.TransferStatus, 'read') |...
    strcmp(sensor_port.TransferStatus, 'write') | ...
    strcmp(sensor_port.TransferStatus, 'read&write') |...
    toc<time)
    drawnow
end

end