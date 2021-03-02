'''
    Adapted from course 16831 (Statistical Techniques).
    Initially written by Paloma Sodhi (psodhi@cs.cmu.edu), 2018
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
'''

import numpy as np
import math

class Resampling:
    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """
    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    def multinomial_sampler(self, X_bar):
        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        """
        TODO : Add your code here
        """
        new_X_bar_indeces = []
        normalized_weights = X_bar[:,-1] / X_bar[:,-1].sum()
        X_bar[:,-1] = 1

        for counter in range(len(X_bar)):
            sampled_idx = np.random.choice(np.arange(len(X_bar)), p = normalized_weights)
            new_X_bar_indeces.append(sampled_idx)
        new_X_bar_indeces = np.array(new_X_bar_indeces)

        X_bar_resampled =  X_bar[new_X_bar_indeces]

        return X_bar_resampled

    def low_variance_sampler(self, X_bar):
        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        """
        TODO : Add your code here
        """
        # np.isfinite(diff_images).all()
        # print(X_bar[:,-1])
        if X_bar[:,-1].sum()==0:
            print("ALL MY WEIGHTS ARE 0")
        normalized_weights = X_bar[:,-1] / X_bar[:,-1].sum()

        X_bar[:,-1] = 1
        
        cum_sum = np.cumsum(normalized_weights)
        even_intervals = np.linspace(0,1,len(X_bar)+1)[1:]
        tiled_intervals = np.tile(even_intervals,(len(X_bar),1)).T
        subtract_2d = cum_sum - tiled_intervals
        
        non_neg_2d = np.where(subtract_2d<-0.00000001, 10, subtract_2d) #0.00000001 accounts for error in np.cum_sum
        low_var_indeces = np.argmin(non_neg_2d,axis=1)

        X_bar_resampled = X_bar[low_var_indeces]
        return X_bar_resampled
    
    def low_variance_decay_sampler(self, X_bar,time_idx):
        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        """
        TODO : Add your code here
        """
        np.isfinite(diff_images).all()
        print(X_bar[:,-1])
        if X_bar[:,-1].sum()==0:
            print("ALL MY WEIGHTS ARE 0")
        normalized_weights = X_bar[:,-1] / X_bar[:,-1].sum()

        X_bar[:,-1] = 1
        
        cum_sum = np.cumsum(normalized_weights)
        even_intervals = np.linspace(0,1,len(X_bar)+1)[1:]
        tiled_intervals = np.tile(even_intervals,(len(X_bar),1)).T
        subtract_2d = cum_sum - tiled_intervals
        
        non_neg_2d = np.where(subtract_2d<-0.00000001, 10, subtract_2d) #0.00000001 accounts for error in np.cum_sum
        low_var_indeces = np.argmin(non_neg_2d,axis=1)
       
        X_bar_resampled = X_bar[low_var_indeces]
        # X_bar_resampled = []
        # M = len(X_bar)
        # wt = X_bar[:,3]
        # r = random.uniform(0, 1.0/M)
        # wt /= wt.sum()
        # c = wt[0]
        # i = 0
        # for m in range(M):
        #     u = r + (m)*(1.0/M)
        #     while u>c:
        #         i = i +1
        #         c = c + wt[i]
        #     X_bar_resampled.append(X_bar[i])
        # X_bar_resampled = np.asarray(X_bar_resampled)
        return X_bar_resampled
        
