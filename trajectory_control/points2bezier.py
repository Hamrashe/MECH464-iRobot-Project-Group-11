import sys
import numpy as np
from scipy.interpolate import BPoly
import bezier


#import movement
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
from scipy import interpolate

cp = np.array( [
    (1,1), (1,5), (5,5), (5,1), (1,1)
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
            print(f'lp is {lp}')
        k = k-1
        
        lp = interp(lp,k)
    return lp

lp = interp(cp,5)



print(cp)
plt.scatter (lp[:,0], lp[:,1])
plt.show()

curve = BPoly(lp[:,np.newaxis,:], [1,5])

x = np.linspace(1,5, 20000)

p = curve(x)
print(f'p is {p.T}')
plt.plot(*p.T)
plt.show()