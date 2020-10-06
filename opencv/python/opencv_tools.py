import numpy as np
import matplotlib.pyplot as plt
import cv2
#%matplotlib inline

## Catalogue of OpenCV functions

# cv2.imread()
# cv2.cvtColor()
# cv2.resize()
# cv2.flip()
# cv2.imwrite()

## Image Handling

def read_image(filename, **kwargs):
    try:
        cmap = kwargs.get('cmap', None)
        img = cv2.imread(filename)
        if cmap == 'gray':
            img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)            
        elif cmap == 'hsv':
            img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        else:
            img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        return img
    except:
        return None

def write_image(img, filename):
    try:
        if len(img.shape) > 2:
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            cv2.imwrite(filename, img)
        cv2.imwrite(filename, img)            
    except:
        pass
    
def display_image(img, size=(10,8)):
    size_cm = (size[0]/2.54, size[1]/2.54)
    fig = plt.figure(figsize=size_cm)
    ax = fig.add_subplot(1,1,1)
    ax.imshow(img)
    
## Image Editing

