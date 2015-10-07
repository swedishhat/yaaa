#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

with open("outfile") as f:
    data = f.read()
    data = data[:-1]

    data = data.split('\n')

    #print(data)

    x = [row.split(' ')[0] for row in data]
    #print(x)
    sins = [row.split(' ')[1] for row in data]
    coss = [row.split(' ')[2] for row in data]
    
    fig = plt.figure()

    ax1 = fig.add_subplot(1,1,1)
    ax2 = fig.add_subplot(1,1,1)

    ax1.set_title("Sine Squared")    
    ax2.set_title("Cosine Squared")    
    ax1.set_xlabel('Pixel Number')
    ax1.set_xlabel('Pixel Number')
    ax1.set_ylabel('Color Value')
    ax1.set_ylabel('Color Value')

    ax1.plot(x,sins, c='r', label='sin')
    ax2.plot(x,coss, c='b', label='cos')
    
    leg = ax1.legend()
    leg = ax2.legend()

    plt.show()
