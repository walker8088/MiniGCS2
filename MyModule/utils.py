#coding utf-8

import sys

if sys.platform == "win32":
    from _winreg import *

    def get_all_comports():
        ports = []
        try:
            key = OpenKey(HKEY_LOCAL_MACHINE,"HARDWARE\\DEVICEMAP\\SERIALCOMM")
        except:
            return []
        try:
            i = 0
            while True:
                name,value,type = EnumValue(key,i)
                ports.append(value)
                i+=1                                 
        except:
            pass
            
        return ports

elif sys.platform == 'linux2':
    def get_all_comports():
	return ("/dev/ttyACM0",)
