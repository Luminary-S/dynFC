ROS driver for magnetic encoder TLE5012B
----------
# wire definition
- blue : +5V  ---> 红
- green:  GND  --->  灰
- yellow: 485A  --->  白
- black: 485B   --->  绿



1. modbus crc16 check, [online tool](https://www.23bei.com/tool-59.html)
2. baud: 38400
3. 485 resolution: 65536




revise ID:
```
# change ID
01 06 00 68 00 02 89 D7
# keep for outage
01 06 00 0A 00 01 68 08 
```
read data 
```
# for ID 1
01 03 00 01 00 01 D5 CA
# for ID 2
02 03 00 01 00 01 D5 F9
```

receive hex:
```
# for ID 1, example, data is "CF C0", angle = 360*(CF*256+C0)/65536, "ED E4" is modbus CRC 16 check
01 03 02 CF C0 ED E4
# for ID 2
02 03 02 56 92 42 49
```

pyserial timeout if too large, the data output will be very slow.