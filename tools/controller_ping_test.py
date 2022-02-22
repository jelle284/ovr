# -*- coding: utf-8 -*-
"""
Created on Sat Mar  6 22:58:56 2021

@author: Jelle
"""

import socket
import time

n_regs = 9
timeout = 0.1
n_packets = 500



sock_addr = '192.168.87.157'
sock_port = 4210
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as s:
    s.settimeout(timeout)
    accum_time = 0
    time.sleep(2)
    req = [1, 0, n_regs, 0]
    dropped = 0
    print("Collecting")
    for i in range(n_packets):
        try:
            tout = time.time()
            s.sendto(bytearray(req), (sock_addr, sock_port))
            res, addr = s.recvfrom(4*n_regs)
            tret = time.time()
            elapsed = tret-tout
            print(".", end='')
            accum_time += elapsed
        except:
            dropped = dropped + 1
    print('!')
    t_avg = accum_time/(n_packets-dropped)
    print(f"Average time: {t_avg*1000:1.2f} ms")
    print("Dropped replies", dropped)