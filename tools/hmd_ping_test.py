# -*- coding: utf-8 -*-
"""
Created on Sat Mar  6 15:12:13 2021

@author: Jelle
"""

import serial
import time
import struct

n_regs = 9
timeout = 0.1
n_packets = 1000
data = []
with serial.Serial('COM5', baudrate = 115200, timeout = timeout) as s:
    accum_time = 0
    time.sleep(2)
    req = [1, 0, n_regs, 0]
    dropped = 0
    print("Collecting")
    for i in range(n_packets):
        tout = time.time()
        s.write(req)
        res = s.read(size=4*n_regs)
        data.append(struct.unpack('9f', res))
        tret = time.time()
        elapsed = tret-tout
        print(".", end='')
        if elapsed > timeout:
            dropped = dropped + 1
        else:
            accum_time += elapsed
    print('!')
    t_avg = accum_time/(n_packets-dropped)
    print(f"Average time: {t_avg*1000:1.2f} ms")
    print("Dropped replies", dropped)
    
import pandas as pd

df = pd.DataFrame(data)
df.plot()