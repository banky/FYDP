import pandas as pd
import numpy as np
import cPickle
import graphlab as gl
import os

dir_path = os.path.dirname(os.path.realpath(__file__))

#Define a function to load each batch as dictionary:
def unpickle(file):
    fo = open(file, 'rb')
    dict = cPickle.load(fo)
    fo.close()
    return dict

#Make dictionaries by calling the above function:
batch1 = unpickle(dir_path + '/data/data_batch_1')
batch2 = unpickle(dir_path + '/data/data_batch_2')
batch3 = unpickle(dir_path + '/data/data_batch_3')
batch4 = unpickle(dir_path + '/data/data_batch_4')
batch5 = unpickle(dir_path + '/data/data_batch_5')
batch_test = unpickle(dir_path + '/data/test_batch')

#Define a function to convert this dictionary into dataframe with image pixel array and labels:
def get_dataframe(batch):
    df = pd.DataFrame(batch['data'])
    df['image'] = df.as_matrix().tolist()
    df.drop(range(3072),axis=1,inplace=True)
    df['label'] = batch['labels']
    return df

#Define train and test files:
train = pd.concat([get_dataframe(batch1),get_dataframe(batch2),get_dataframe(batch3),get_dataframe(batch4),get_dataframe(batch5)],ignore_index=True)
test = get_dataframe(batch_test)

print(type(train))
print(train.head())
print(train.shape, test.shape)

gltrain = gl.SFrame(train)
gltest = gl.SFrame(test)

#Convert pixels to graphlab image format
gltrain['glimage'] = gl.SArray(gltrain['image']).pixel_array_to_image(32, 32, 3, allow_rounding = True)
gltest['glimage'] = gl.SArray(gltest['image']).pixel_array_to_image(32, 32, 3, allow_rounding = True)

#Remove the original column
gltrain.remove_column('image')
gltest.remove_column('image')

#Convert into 256x256 size
gltrain['image'] = gl.image_analysis.resize(gltrain['glimage'], 256, 256, 3)
gltest['image'] = gl.image_analysis.resize(gltest['glimage'], 256, 256, 3)

#Remove old column:
gltrain.remove_column('glimage')
gltest.remove_column('glimage')

print(gltrain.head())

#Load the pre-trained model:
pretrained_model = gl.load_model('http://s3.amazonaws.com/GraphLab-Datasets/deeplearning/imagenet_model_iter45')

gltrain['features'] = pretrained_model.extract_features(gltrain)
gltest['features'] = pretrained_model.extract_features(gltest)

print(gltrain.head())

simple_classifier = gl.classifier.create(gltrain, features = ['features'], target = 'label')

# model = gl.neuralnet_classifier.create(gltrain, target='label', validation_set=None)
# model.evaluate(gltest)
