
import csv
import numpy as np
from scipy.misc import imread
from scipy.misc import imresize
import pickle


# In[3]:

# this file is used to process the raw output of recording training data from the simulator
# and prepare it for uploading to the AWS GPU

folder_to_process = "output"
print("Processing ", folder_to_process)


def extract_data(folder):
    X = [] #images
    XX = [] #senor data (aka speed in this case)
    y = [] #actual steering angles (throttle in the future)


    with open('/Users/watson/output/_data.csv') as f:
        reader = csv.reader(f)

        for line in reader:
            img_file_center = "{}_{}".format(int(line[0]),line[-1])

            X.append(imread('/Users/watson/output/{}.jpg'.format(img_file_center)))
            XX.append([float(line[1])/100]) #throttle is -100 to 100
            y.append([float(line[2])/45]) # steering is -45 to 45

    return (np.array(X), np.array(XX), np.array(y))


X_train, XX_train, y_train = extract_data(folder_to_process)

print("--done--")


# In[4]:

def pipeline(img):
    # img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)

#     # equalize the histogram of the Y channel
#     img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])

#     # convert the YUV image back to RGB format
#     img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
#     print(img.shape)

    output = img[200:361,:,:]
    return  imresize(output, .5)  

# reduce image to the road section and apply any transforms in the pipeline
X_train = np.array([pipeline(x) for x in X_train])


# In[7]:

X_train.shape


data_frames_to_drop = [(0,1800), (2080, 2210),(2300, 2590), (2790,2950), 
                       (3400,4060), (4220, 4560), (5110,5230, (5670, 7085))]


clean_indicies = np.ones(len(X_train))
for r in data_frames_to_drop:
    clean_indicies[r[0]:r[1]] = 0


# In[9]:

X_cleaned = X_train[clean_indicies.astype(np.bool)]
XX_cleaned = XX_train[clean_indicies.astype(np.bool)]
y_cleaned = y_train[clean_indicies.astype(np.bool)]

print(X_cleaned.shape,XX_cleaned.shape, y_cleaned.shape)

data = {
    "images": X_train,
    "sensors": XX_train,
    "steering_throttle": y_train
}


with open('vector79_run_1.pkl', 'w') as f:
    pickle.dump(data, f)


# In[ ]:



