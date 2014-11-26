import os, sys
import Image
import numpy as np

def getNumPixels(image):
    width, height = image.size
    return width, height

def makeHeightArray(image_path):
    image = Image.open(image_path)
    image_pixels = image.load()
    width, height = getNumPixels(image)
    print "height: ", height, " width: ", width

    height_array = []
    for i in range(width):
        for j in range(height):
            if (image_pixels[i,j] == (0,0,0)):
                height_array.append(j)
                break

    return height_array

def saveArray(height_array):
    X = []
    for idx, height in enumerate(height_array):
        X.append([idx, height])
    np.savetxt('../../../../debug/crew/Pilots/ObstacleAvoidancePilot/bottom.txt', X, fmt='%d', delimiter=" ")

if __name__ == '__main__':

    if len(sys.argv) == 2:
        image_path = sys.argv[1]
    else:
        print "Usage: python generate_bottom.py <image_path.jpg>"
        exit()

    array = makeHeightArray(image_path)
    print "array: ", array
    print "size of array: ", len(array)
    
    # TODO: delete this:
    # ----------------- #
    depth = 10
    size = 400
    array = []
    for i in range(size):
        if (i == round(size/2.0)):
            depth = depth*0.7
        array.append(depth)
    print "This one overwrites the other"
    print array
    print "size: ", size
    # ----------------- #
    
    saveArray(array)
