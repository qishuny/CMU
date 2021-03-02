import cv2
import numpy as np
import glob
 
# img_array = []

# # for i in range(2218):
    
# #     # img = cv2.imread('./results/*.jpg'))
# #     img = cv2.imread('./results/'+str(i).zfill(4)+'.png')
# #     print(str(i).zfill(4))
# #     height, width, layers = img.shape
# #     size = (width,height)
# #     img_array.append(img)


# for filename in glob.glob('C:/Users/Yuqis/OneDrive/16833/problem_set/code/results/*.png'):
#     img = cv2.imread(filename)
#     height, width, layers = img.shape
#     size = (width,height)
#     img_array.append(img)
# out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 5, size)
 
# for i in range(len(img_array)):
#     out.write(img_array[i])
# out.release()

# import os
# import cv2
# import time


import os
import cv2
import time


def picvideo(path, size):
    filelist = os.listdir(path)


   
    fps = 30

    file_path = "C:/Users/Yuqis/OneDrive/16833/problem_set/code" + str(int(time.time())) + ".avi"  
    fourcc = cv2.VideoWriter_fourcc('I', '4', '2', '0')  

    video = cv2.VideoWriter(file_path, fourcc, fps, size)
    progress=0
    for item in filelist:
        if item.endswith('.png'):  
            item = path + '/' + item
            img = cv2.imread(item)

            video.write(img)  
        progress+=1
        if progress%100==0:
            print('progress',progress/2217)
    video.release()

path ='C:/Users/Yuqis/OneDrive/16833/problem_set/code/results'
size = (640,480) 
picvideo(path,size)