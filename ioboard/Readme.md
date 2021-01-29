4 routes IO board
-------------------

[pump link](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.3a842e8d4LOX0N&id=12877399994&_u=m35ibqcp24ac)

port 1 is water inlet; port 2 is water sewage outlet. 

open or close one port
```
0100, 01 is port 1, next 01 is linked  00 is unlinked
port 1, close
55 C8 01 00 55
return
1B DB 01 00 0C
port 1, open
55 C8 01 00 55

port 2, close
55 C8 02 00 55
port 2, open
55 C8 02 00 55

port all open 
55 C8 09 0F 55
return
1B DB 00 0F 0C

port all close
55 C8 09 00 55
return
1B DB 00 00 0C
```

8421 code for simultaneously control all ports:
1. port1: 1; port2: 2; port3: 4; port4: 8
2. if port1 and port3 open, byte4 should be 0x05