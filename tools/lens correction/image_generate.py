# -*- coding: utf-8 -*-
"""
Created on Sun Jul 12 23:00:37 2020

@author: Jelle
"""


import numpy as np

xc, yc = 0.5, 0.5

line_thick = 3
line_spacing = 100
line_col_d = np.array((0,0,0), 'uint8')
dotsize = 4

def brown(xd,yd, k1, k2, p1, p2):
    r = np.sqrt((xd-xc)**2 + (yd-yc)**2)
    coeff = (k1*r**2+k2*r**4)
    xu = xd + (xd-xc)*coeff + (p1*(r**2+2*(xd-xc)**2) + 2*p2*(xd-xc)*(yd-yc))
    yu = yd + (yd-yc)*coeff + (p2*(r**2+2*(xd-xc)**2) + 2*p1*(xd-xc)*(yd-yc))
    return xu, yu

def radial(xd,yd, k1, k2):
    r = np.sqrt((xd-xc)**2 + (yd-yc)**2)
    coeff = 1 + k1*r**2 + k2*r**4
    xu = xc + (xd-xc)/coeff
    yu = yc + (yd-yc)/coeff
    return xu, yu
    
def get_blank(width, height):
    return np.full((width,height,3), 255, 'uint8')


def hline(row, width, weight):
    return np.array([[row+t,c] for c in range(width) for t in range(weight)])

def vline(col, height, weight):
    return np.array([[r,col+t] for r in range(height) for t in range(weight)])

def draw_grid(ni, k1, k2, p1=0, p2=0):
    h, w, c = ni.shape
    
    lines = []
    # horizontal lines
    for line in range(int((h/2)/line_spacing)):
        upper = int(h/2 + (line+0.5)*line_spacing)
        lower = int(h/2 - (line+0.5)*line_spacing)
        lines.append(hline(upper, w, 3))
        lines.append(hline(lower, w, 3))
        
    # vertical lines 
    for line in range(int((w/2)/line_spacing)):
        right = int(w/2 + (line+0.5)*line_spacing)
        left = int(w/2 - (line+0.5)*line_spacing)
        lines.append(vline(right, h, 3))
        lines.append(vline(left, h, 3))
    
    for l in lines:
        for x,y in l:
            xd, yd = x/h, y/w
            xu, yu = brown(xd, yd, k1, k2, p1, p2)
            xp, yp = int(xu*h), int(yu*w)
            if (xp in range(h)) and (yp in range(w)):
                ni[xp, yp, :] = line_col_d
    
    midh, midw = (int(h/2), int(w/2))
    ni[midh-dotsize:midh+dotsize, midw-dotsize:midw+dotsize, :] = 0
    return ni
        