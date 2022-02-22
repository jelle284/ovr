# -*- coding: utf-8 -*-
"""
Created on Tue Feb 16 19:34:49 2021

@author: Jelle
"""

import serial
import socket
import time
import struct
import pandas as pd
from matplotlib import pyplot as plt

SAMPLE_REQ = [1,16,2,0]
SAMPLE_SIZE = 200

def sample_hmd():
    with serial.Serial('COM5', baudrate = 115200, timeout = 1.0) as s:
        time.sleep(2)
        
        data = []
        
        print("Gathering samples")
        for _ in range(SAMPLE_SIZE):
            s.write(SAMPLE_REQ)
            res = s.read(size=8)
            if len(res) == 8:
                sample = [m for m in struct.unpack('hhhh', res)[:3]]
                data.append(sample)
            print(".", end="")
            time.sleep(0.1)
            
        print("Finished")
        
    
    df = pd.DataFrame(data, columns=['x','y','z'])
    
    return df

def sample_controller():
    sock_addr = '192.168.87.165'
    sock_port = 4210
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as s:
        s.settimeout(5.0)
        data = []
        print("Gathering samples")
        for _ in range(SAMPLE_SIZE):
            s.sendto(bytearray(SAMPLE_REQ), (sock_addr, sock_port))
            try:
                res, addr = s.recvfrom(32)
            except socket.timeout:
                pass
            else:
                if len(res) == 8:
                    sample = [m for m in struct.unpack('hhhh', res)[:3]]
                    data.append(sample)
                print(".", end="")
                time.sleep(0.1)
        print("Finished")

    df = pd.DataFrame(data, columns=['x','y','z'])
    
    return df

def calibrate(df):
    bias = [(df.x.max() + df.x.min())/2,
            (df.y.max() + df.y.min())/2,
            (df.z.max() + df.z.min())/2]
    scale = [(df.x.max() - df.x.min())/2,
             (df.y.max() - df.y.min())/2,
             (df.z.max() - df.z.min())/2]
    scale_avg = sum(scale)/len(scale)
    for i in range(3):
       scale[i] /= scale_avg
    return bias, scale

def magplot(df):
    plt.plot(df.x, df.y, 'o')
    plt.plot(df.y, df.z, 'o')
    plt.plot(df.z, df.x, 'o')
    
if __name__ == '__main__':
    df = sample_controller()
    bias, scale = calibrate(df)
    magplot(df)
    print("bias is:", bias)
    print("scale is:", scale)