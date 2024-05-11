#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
from sklearn.linear_model import LinearRegression
import random

class PredictiveCleaner:
    def __init__(self):
        rospy.init_node('predictive_cleaner', anonymous=True)
        self.model = LinearRegression()
        self.cleaned_data = []
        
        self.prediction_pub = rospy.Publisher('/predicted_dirt_location', Point, queue_size=10)
        rospy.Subscriber('/cleaned_location', Point, self.cleaned_callback)

    def cleaned_callback(self, msg):
        self.cleaned_data.append([msg.x, msg.y])
        rospy.loginfo(len(self.cleaned_data))
        if len(self.cleaned_data) >= 2:
            self.train_model()

    def train_model(self):
        data = np.array(self.cleaned_data)
        if data.shape[0] > 1:
            X = data[:, :1]
            y = data[:, 1]
            self.model.fit(X, y)
            self.predict_and_publish()

    def predict_and_publish(self):
        if hasattr(self, 'model'):
            x_new = np.array([[random.uniform(-5, 5)]])
            y_pred = self.model.predict(x_new)
            self.prediction_pub.publish(Point(x=x_new[0, 0], y=y_pred[0], z=0))

if __name__ == '__main__':
    try:
        pc = PredictiveCleaner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

