import numpy as np
from collections import defaultdict 
class ReadTable:
    def __init__(self):
        with open('lmap.npy','rb') as f:
            self.raymap = np.load(f)
        
        # self.raymap = defaultdict(lambda: defaultdict(dict))
                
        # for i in range(800):
        #     x_val = 3000 +i*5
        #     for j in range(1600):
        #         y_val = j*5
        #         for k in range(72):
        #             theta_val = k*5
        #             self.raymap[x_val][y_val][theta_val] = load_original_arr[i][j][k]
        #     print("process",i/800*100)
            # print(raymap[4000][5000][0])
            # print(raymap[4000][5000][90])
            # print(raymap[4000][5000][180])
            # print(raymap[4000][5000][270])
            
    def return_Map(self):
        return self.raymap