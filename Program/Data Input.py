import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.utils import to_categorical
from tensorflow.keras import layers
from sklearn.model_selection import train_test_split
import keras
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Activation
from keras.layers import Dropout
from keras.layers import LSTM
from tensorflow.keras import backend as K
from tensorflow.keras.callbacks import Callback
import matplotlib.pyplot as plt

def lms(y_true,y_pred): #by simon haykin book's
  return (K.square(y_pred-y_true))/2

# Data Predict
# Masukan Input disini
loaded_model = tf.keras.models.load_model("ModelNN.h5", custom_objects={"lms": lms })
data = {
    'x': 0.072,
    'y': 0.219,
    'z': 0.365,
    'roll': 0.001,
    'pitch': 1.047,
    'yaw': 1.31
}

# Create a DataFrame with a single row
df = pd.DataFrame(data, index=[0])

Y_prediksi = loaded_model.predict(df)
print(Y_prediksi)