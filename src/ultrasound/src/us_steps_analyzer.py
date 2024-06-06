#! /usr/bin/env python3

from us_static_analyzer import load_bag

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from scipy.stats import norm
from scipy.stats import shapiro


def load():
    STEPS_TIMINGS = np.array([
        #(  d,  tbeg,  tend)
        (0.05,   0.0,  17.0),
        (0.10,  20.5,  32.0),
        (0.15,  34.0,  49.5),
        (0.20,  51.8,  64.0),
        (0.25,  66.0,  77.5),
        (0.30,  79.3,  95.3),
        (0.35,  96.5, 110.0),
        (0.40, 114.0, 126.8),
        (0.45, 128.7, 143.1),
        (0.50, 145.0, 157.0),
        (0.55, 157.7, 174.2),
        (0.60, 175.0, 188.0),
        (0.65, 189.8, 203.0),
        (0.70, 204.2, 217.4),
        (0.75, 218.5, 231.8),
        (0.80, 232.8, 246.0),
        (0.85, 246.7, 259.6),
        (0.90, 262.2, 275.0),
        (0.95, 276.3, 290.0),
        #(1.00, 290.5, 305.0),
        #(1.05, 306.0, 319.0),
        #(1.10, 320.0, 329.0),
    ])

    times, values = load_bag()
    step_filters = [
        (d, (times>tbeg) & (times<tend)) for d, tbeg, tend in STEPS_TIMINGS
    ]

    return np.hstack([
        [times[filt]-times[0], values[filt], np.full(np.sum(filt), d)] for d, filt in step_filters
    ]).T

if __name__=="__main__":
    steps = load()
    times=steps[:,0]
    values=steps[:,1]
    #plt.plot(times,values) #augmentation du bruiten fonction de la distance 
    #plt.show()
    
    theorical_values=steps[:,2]

    alpha = (sum((values-values.mean())*(theorical_values-theorical_values.mean())))/(sum((theorical_values-theorical_values.mean())**2))
    beta = values.mean()-alpha*theorical_values.mean()
    
    print (alpha, beta)
    
    plt.plot(times, values)
    plt.plot(times, alpha*theorical_values+beta)
    err=values-alpha*theorical_values+beta
    plt.plot(times, err)
    err_d=(err-beta)/alpha
    plt.show()
    
    plt.hist(err, density=True)
    plt.show()

    plt.hist(err_d, density=True)
    plt.show()
    
    var_d=values.var()/(alpha**2)
    print("var_d",var_d)
    print("err_d",err_d.mean())
    

    
    # steps is a N*3 np.array like
    # | time1 | measurement1 | theorical_distance1 |
    # | time2 | measurement2 | theorical_distance2 |
    # | time3 | measurement3 | theorical_distance3 |

    # TODO: Analysis
