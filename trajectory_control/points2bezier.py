import sys
import numpy as np
from scipy.interpolate import BPoly
import bezier
from sympy import *


#import movement
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
from scipy import interpolate

cp = np.array( [
     (0,0), (0,100), (100,100)
], dtype=float)
i = 0
lp = cp
index = 0
def interp(arr, k):
    lp = arr
    if k != 0:
        for i in range(len(lp)-1):
            x1 = arr[i,0]
            x2 = arr[i+1,0]
            y1 = arr[i,1]
            y2 = arr[i+1,1]
            x = (x1 + x2)/2
            y = (y1 + y2)/2
            print((x,y))
            
            lp =np.insert(lp,int((i+0.5)*2),(x,y), axis=0)
            #print(f'lp is {lp}')
        k = k-1
        
        lp = interp(lp,k)
    return lp


def main():



    lp = interp(cp,1)
    s = symbols('s')
    print(bezier.Curve.from_nodes(lp.T).to_symbolic()[0])
    coeffs = np.polyfit(lp[:,0], lp[:,1], 9)
    eqn = 0
    index = 0
    for coef in coeffs:
        eqn = coef*s**index



    print(cp)
    plt.scatter (lp[:,0], lp[:,1])
    plt.show()

    curve = BPoly(lp[:,np.newaxis,:], [1,5])

    x = np.linspace(1,5, 20000)

    print(curve)
    p = curve(x)
    #s_t = curve(s)

    #print(s_t)
    print(f'p is {p.T}')
    plt.plot(*p.T)
    plt.show()
    return bezier.Curve.from_nodes(lp.T).to_symbolic()

#print(main()[0])