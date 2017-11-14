import os
import sys
import cv2
import tensorflow as tf
import pandas as pd
from PIL import Image
import io
import PIL.Image
import numpy as np

# from object_detection.utils import dataset_util

csv_path = os.path.join(os.path.dirname(sys.path[0]),"datasets", "csv_files", "data.csv")
img_path = os.path.join(os.path.dirname(sys.path[0]),"datasets", "images")
out_dir = os.path.join(os.path.dirname(sys.path[0]),"datasets", "outdir")
if not os.path.exists(out_dir):
    os.makedirs(out_dir)
def get_csv_data(file):
    data = pd.read_csv(file)
    image_names, xmin, ymin, xmax, ymax = [], [], [], [], []
    image_names = list(data['filename'])
    xmin = list(data['xmin'])
    ymin = list(data['ymin'])
    xmax = list(data['xmax'])
    ymax = list(data['ymax'])
    return image_names, xmin, ymin, xmax, ymax


if __name__ == "__main__":

    image_names, xmin, ymin, xmax, ymax = get_csv_data(csv_path)
    for index in range(len(image_names)):
        filename = image_names[index]
        filepath = os.path.join(img_path, image_names[index])
        img = cv2.imread(filepath)

        cv2.rectangle(img,(xmin[index], ymin[index]),(xmax[index], ymax[index]),(0,255,0),3)
        cv2.imwrite(out_dir+"/"+filename,img)
    print("Done writing labelled images to `datasets/outdir`")
