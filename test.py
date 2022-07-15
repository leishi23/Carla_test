# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
# import glob
# import pylab as pl

####################################### Lidar ######################################
# dir_path = "/home/lshi23/research_course_code/rgb_out/*.*"
# res = glob.glob(dir_path) # get the image path
# res.sort()
# img = None
# for i in res:
#     image_path = i
#     im = mpimg.imread(image_path)
#     if img is None:
#         img = pl.imshow(im)
#     else:
#         img.set_data(im)
#     pl.pause(.05)
#     pl.draw


####################################### Depth ######################################
# dir_path = "/home/lshi23/research_course_code/depth_out/*.*"
# res = glob.glob(dir_path) # get the image path
# res.sort()
# img = None
# for i in res:
#     image_path = i
#     im = mpimg.imread(image_path)
#     if img is None:
#         img = pl.imshow(im,cmap='gray')
#     else:        
#         img.set_data(im)
#     pl.pause(.05)
#     pl.draw

import time
a = time.time()
print(a)
print(time.ctime(a))
print(time.ctime((a+1)))
