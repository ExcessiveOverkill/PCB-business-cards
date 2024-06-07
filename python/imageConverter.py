import cv2
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilenames
import os




ENABLE_ANGLE_STABILIZATION_MODE = False
RANDOM_CHANCE = 10  # max is 0x3FFF
RANDOM_POINTS = 1


# number of side/side movements to display each frame for
ANIMATION_FRAME_DISPLAY_CYCLES = [1, 1]   # normal and secret frames, how many times to display a frame when it is loaded from an animation
SINGLE_FRAME_DISPLAY_CYCLES = [16, 4]   # normal and secret frames, how many times to display a frame when it is loaded along
SECRET_DISPLAY_CYCLES = 1   # how many full cycles to display the secret image/animation for




def distortion(x):
    return ((x-1)**4) * 2

Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing

normalFileNames = askopenfilenames(title="Images to display normally", filetypes=[("Image files", "*.png")], initialdir=os.path.dirname(os.path.realpath(__file__)))
secretFileNames = askopenfilenames(title="Secret images to display randomly", filetypes=[("Image files", "*.png")], initialdir=os.path.dirname(os.path.realpath(__file__)))

if (len(normalFileNames) == 0):
    print("No normal images selected, exiting")
    exit()

combinedImages = [None]
imageDisplayCycles = [[]]
enable_secret_mode = False
if (len(secretFileNames) != 0):
    enable_secret_mode = True
    combinedImages = [None, None]
    imageDisplayCycles = [[], []]


for imageType in range(len(combinedImages)):
    for imageFile in [normalFileNames, secretFileNames][imageType]:

        # load image and convert to binary mask
        image = cv2.imread(imageFile)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if (image.shape[0] != 32):  # resize image if it is not 32 high
            image = cv2.resize(image, (int(32/image.shape[0] * image.shape[1]), 32), interpolation= cv2.INTER_NEAREST)
            print(f"Image ({imageFile}) resized to {image.shape[1]} x {image.shape[0]}")


        image = cv2.flip(image, 0)
        ret, image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)
        image = image.T     # flip array so we can index by columns first

        if(image.shape[1] != 32):
            print(f"Image height must be 32, not {image.shape[1]}. exiting...")
            exit()

        # check if image has animation separators
        isAnimation = False
        for index, line in enumerate(image):
            if(np.all(line == [255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0]) or np.all(line == [0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255])):
                isAnimation = True
                if(imageDisplayCycles[imageType] is None):
                    imageDisplayCycles[imageType] = [ANIMATION_FRAME_DISPLAY_CYCLES[imageType]]
                    continue
                imageDisplayCycles[imageType].append(ANIMATION_FRAME_DISPLAY_CYCLES[imageType])

        
        if (not isAnimation):
            # crop image since it is not an animation
            startIndex = min(np.nonzero(image)[0])
            endIndex = max(np.nonzero(image)[0])
            image = image[startIndex:endIndex+1]
            if(imageDisplayCycles[imageType] is None):
                imageDisplayCycles[imageType] = [SINGLE_FRAME_DISPLAY_CYCLES[imageType]]
            imageDisplayCycles[imageType].append(SINGLE_FRAME_DISPLAY_CYCLES[imageType])
        else:
            imageDisplayCycles[imageType].append(ANIMATION_FRAME_DISPLAY_CYCLES[imageType])


        # add image
        if(combinedImages[imageType] is None):
            combinedImages[imageType] = image
            continue
        separator = np.array([255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0], dtype=np.uint8)
        separator = np.reshape(separator, (1, 32))
        combinedImages[imageType] = cv2.vconcat([combinedImages[imageType], separator, image])

    #cv2.imshow("Combined frames", combinedImages[imageType])
    #cv2.waitKey(0)





convertedBits = []

displayFrameWidth = [0, 0]

for imageIndex, combinedImage in enumerate(combinedImages):

    # split into frames by searching for vertical dotted lines
    frames = []
    lastSeparatorLine = 0
    biggestFrameLength = 0
    for index, line in enumerate(combinedImage):
        if(np.all(line == [255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0]) or np.all(line == [0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255])):
            frames.append(combinedImage[lastSeparatorLine:index])
            biggestFrameLength = max(biggestFrameLength, len(frames[-1]))
            lastSeparatorLine = index+1
    

    # add last frame or the only frame if there are no separators
    frames.append(combinedImage[lastSeparatorLine:])
    biggestFrameLength = max(biggestFrameLength, len(frames[-1]))

    if(len(frames) == 1):
        startIndex = min(np.nonzero(frames[0])[0])
        endIndex = max(np.nonzero(frames[0])[0])
        frames[0] = frames[0][startIndex:endIndex+1]
        biggestFrameLength = len(frames[0])


    # add black buffer to frames to ensure they are all equal width
    for index, frame in enumerate(frames):
        additionalBuffer = biggestFrameLength - len(frame)
        leftBuffer = np.floor(additionalBuffer/2)
        rightBuffer = additionalBuffer - np.floor(additionalBuffer/2)
        frames[index] = cv2.vconcat([np.zeros(shape=(int(leftBuffer), 32), dtype=np.uint8), frame, np.zeros(shape=(int(rightBuffer), 32), dtype=np.uint8)])
    

    # convert pixel columns to 32-bit integers
    bitArray = []
    demoImage = []

    for frame in frames:
        
        for columnIndex, column in enumerate(frame):
            tempData = 0
            for pixel in range(0, 32):
                if column[pixel]:
                    tempData += 2**pixel

            x = abs(2*(columnIndex/(len(frame)-1) - 1))
            
            undistortStretches = int(distortion(x)) + 1
            for i in range(undistortStretches):
                demoImage.append(column)
                bitArray.append(tempData)

        if (displayFrameWidth[imageIndex] == 0):
            displayFrameWidth[imageIndex] = len(demoImage)

    x = 0
    while(x < 2):
        for frameNumber in range(len(demoImage)//displayFrameWidth[imageIndex]):
            image = np.flip(np.array(demoImage[frameNumber*displayFrameWidth[imageIndex]:(1+frameNumber)*displayFrameWidth[imageIndex]]).T, axis=0)
            image = cv2.resize(image, (displayFrameWidth[imageIndex]*16, 32*16), interpolation= cv2.INTER_NEAREST)
            if(imageType == 0):
                cv2.imshow("Normal image distorted frame", image)
            else:
                cv2.imshow("Secret image distorted frame", image)
            cv2.waitKey(100 * imageDisplayCycles[imageIndex][frameNumber])
        x += 1

    convertedBits.append(bitArray)


# write to file
fileData = f"#define IMAGE_DATA {{{str(convertedBits[0])[1:-1]}}}\n#define IMAGE_LENGTH {len(convertedBits[0])}\n#define FRAME_LENGTH {displayFrameWidth[0]}\n#define FRAME_DISPLAY_CYCLES {{{str(imageDisplayCycles[0])[1:-1]}}}\n"

if (enable_secret_mode):
    fileData += f"#define SECRET_IMAGE_DATA {{{str(convertedBits[1])[1:-1]}}}\n#define SECRET_IMAGE_LENGTH {len(convertedBits[1])}\n#define SECRET_FRAME_LENGTH {displayFrameWidth[1]}\n#define SECRET_FRAME_DISPLAY_CYCLES {{{str(imageDisplayCycles[1])[1:-1]}}}\n"

if (ENABLE_ANGLE_STABILIZATION_MODE):
    if (ANIMATION_FRAME_DISPLAY_CYCLES[0] > 1 or ANIMATION_FRAME_DISPLAY_CYCLES[1] > 1 or SINGLE_FRAME_DISPLAY_CYCLES[0] > 1 or SINGLE_FRAME_DISPLAY_CYCLES[1] > 1):
        print("Angle stabilization cannot be used with multiple display cycles, exiting")
        exit()

    if (enable_secret_mode and len(convertedBits[0])/displayFrameWidth[0] != len(convertedBits[1])/displayFrameWidth[1]):
        print("Angle stabilization cannot be used with secret mode if it doesn't have a matching number of frames as the normal image, exiting")
        exit()

    fileData += f"#define ENABLE_ANGLE_STABILIZATION_MODE 1\n"

fileData += f"#define RANDOM_CHANCE {RANDOM_CHANCE}\n#define RANDOM_POINTS {RANDOM_POINTS}\n#define SECRET_DISPLAY_CYCLES {SECRET_DISPLAY_CYCLES}\n"

f = open("STM32/opensauce 2024/opensauce2024/Core/Inc/imageData.h", "w")
f.write(fileData)
f.close()

print("Image data converted and saved")

#print(fileData)

