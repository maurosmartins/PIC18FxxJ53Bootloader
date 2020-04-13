# The MIT License (MIT)
# Copyright (c) 2016 Vladimir Ignatev
#
# Permission is hereby granted, free of charge, to any person obtaining 
# a copy of this software and associated documentation files (the "Software"), 
# to deal in the Software without restriction, including without limitation 
# the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the Software 
# is furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included 
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
# OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import sys
from enum import IntEnum
import time
import serial
from serial.tools import list_ports
import argparse

class sysStates(IntEnum):
    sWaitingBootloader = 0
    sSendingData = 1
    sError = 2
    sEnd = 3

def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))
    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)
    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush()  # As suggested by Rom Ruben (see: http://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console/27871113#comment50529068_27871113)

def WaitingBootloader():
    global currentState
    
    print('Contacting Bootloader')
    currentState = sysStates.sSendingData
    
    sp.write( bytes('B','UTF-8') )
    time.sleep(0.1)
    bytesread = sp.read()

    try:
        bytesread = bytesread.decode("utf-8")
    except UnicodeDecodeError:
        print("exception detected")
        bytesread = ''
    
    if( (len(bytesread) != 1) or (bytesread!='A') ):
        currentState = sysStates.sError
    else:
        #print(f"received {bytesread}")
        currentState = sysStates.sSendingData
    return
    
    
def SendingData():
    global currentState
    #print('Sending Data')
    f = open(args.bin, "rb")
    data = list(f.read())
    #data = f.read()
    
    progsize = f.seek(0,2)                   #check how long is the binary file
    
    #print("writting length")
    #print(f"Datalen - {progsize}")
    print("Erasing FLASH Memory")
    bytesdatalen = (progsize-0x400-8).to_bytes(4, byteorder='little')
    #print(bytesdatalen)
    sp.write(bytesdatalen)   
    #wait confirmation
    bytesread = sp.read()
    try:
        #print(f"confirmation len {bytesread}")
        bytesread = bytesread.decode("utf-8")
    except UnicodeDecodeError:
        print("exception detected data len")
        bytesread = ''
        
    if( (len(bytesread) != 1) or (bytesread!='B') ):
        #print(bytesread)
        currentState = sysStates.sError
        return
    #else:
        
        #print(f"received {bytesread}")
        #print("length correctly written")
        

    #print(f"Datalen = {progsize}")
    
    print(f"Programming Memory 0x400 - {hex(progsize-8)}")
    
    step = 64
    time.sleep(0.01)
    counter = 0
    total = int( (progsize-0x400-8)/step )
    
    
    for i in range(0x400,progsize-8, step): 
        
        #print(f"{counter} of {(progsize-0x400-8)/step}")
        counter = counter + 1
        datachunk = data[i:i+step]
        #print(datachunk)
        sp.write( len(datachunk).to_bytes(1, byteorder='little') )
        sp.write(bytearray(datachunk))
        #wait confirmation
        bytesread = sp.read()               #this has a timeout
        try:                                #check if 'K' (ok) was received acknowledging the reception of the packet
            bytesread = bytesread.decode("utf-8")
        except UnicodeDecodeError:
            print("exception detected")
            bytesread = ''
        if( (len(bytesread) != 1) or (bytesread!='C') ):
            print("Error Writting Data")
            currentState = sysStates.sError
            break
        else:
            progress(counter, total, args.bin)
            #print(f"received {bytesread}")
            #print("Data chunk correctly written")
            #time.sleep()
            

    currentState = sysStates.sEnd
    return
    
def Error():
    global currentState
    
    print('Error')
    currentState = sysStates.sEnd
    return
    
    
    
def switch(i):
    switcher={
        0:WaitingBootloader,
        1:SendingData,
        2:Error,
    }
    func = switcher.get(i,lambda :'Invalid')
    return func()
    
    
if __name__ == '__main__':

    parser = argparse.ArgumentParser()                                          #creates new argparse instance
    parser.add_argument("bin", help="binary file (.bin) with code to upload")   #adds required arguments 
    parser.add_argument("-b", help="baudrate (default = 19200)", type = int)    #optional argument for baud rate    
    args = parser.parse_args()                                                  #parses the arguments passed to the program
    #print(args.bin)                                                             #how to access arguments (by name)
    #if args.b:
    #    print(args.b)
    
    
    
    global currentState
    currentState = sysStates.sWaitingBootloader
    
    p = list(list_ports.grep("10C4:EA60")) #CP2101 - Silicon Labs VID:PID
    if len(p)==0:
        print("No Serial Port Detected")
        currentState = sysStates.sError
    else:
        sp  = serial.Serial(port=p[0][0], baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=5, xonxoff=False, rtscts=False, write_timeout=None, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
    
    
    
    while True:
        
        switch(currentState)
        if (currentState == sysStates.sEnd):
            try:
                sp.close()
            except NameError:
                print("...")
                
            print('End')
            break
        
    
    
    
