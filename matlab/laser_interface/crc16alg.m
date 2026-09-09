function crc16 = crc16alg (msg)

CRC16_GEN_POL = 32773; % 0x8005
crc16 = 0;
abData(1) = 0;
len = length(msg);

for f=1:len
    abData(2) = abData(1);
    abData(1) = msg(f);
    if crc16 > 32768 % (crc16 & 0x8000)
        crc16 = mod(crc16,32768) * 2;
                % crc16 = (crc16 & 0x7fff) << 1;
        crc16 = binvec2dec(xor(dec2binvec(crc16,24),...
                dec2binvec(CRC16_GEN_POL,24)));
    else
        crc16 = crc16 * 2; % crc16 <<= 1;
    end;
    % crc16 ^= MKSHORT(abData[0], abData[1]);
    abData(2) = mod(abData(2),256) * 256;
    crc16 = binvec2dec(xor(dec2binvec(crc16,24),...
        (dec2binvec(abData(1),24) |...
        dec2binvec(abData(2),24))) );
end;

