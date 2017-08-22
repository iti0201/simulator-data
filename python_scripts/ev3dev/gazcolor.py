import rospy
import time
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image


class GazColor:
    def __init__(self):
        self.image_data = None
        self.image_height = 0
        self.image_width = 0
        self.color_dict = {}
        self.raw_total = None
        self.ready = False
        # while self.image_height == 0:  # when you boot up gazebo and then subscribe to the topic it doesn't like to answer for the first time
        self.sub = rospy.Subscriber('/camera1/image_raw', Image, self.callback)

    # gazebo_msgs/Image
    def callback(self, data):
        while True:
            if data.data is not None:
                self.image_data = data.data
                self.image_height = data.height
                self.image_width = data.width
                self.get_totals()
                break

    def listener_color(self):
        while True:
            if self.ready:
                return self.get_image_color()

    def listener_raw_color(self):
        while True:
            if self.ready:
                return self.get_image_raw_color()

    def listener_reflect(self):
        while True:
            if self.ready:
                return self.get_image_reflect()

    def get_image_color(self):
        color_dict = self.color_dict
        if color_dict is None:
            return 0

        dominant_color = max(color_dict, key=color_dict.get)
        if dominant_color == 'none':
            return 0
        elif dominant_color == 'black':
            return 1
        elif dominant_color == 'blue':
            return 2
        elif dominant_color == 'green':
            return 3
        elif dominant_color == 'red':
            return 5
        elif dominant_color == 'white':
            return 6
        else:
            return 0

    def get_totals(self):
        image_height = self.image_height
        image_width = self.image_width
        image_data = self.image_data

        red_total = 0
        green_total = 0
        blue_total = 0
        total_pixels = 0

        color_dict = {'black': 0, 'white': 0, 'red': 0, 'green': 0, 'blue': 0, 'none': 0}
        radius = min(image_height, image_width) / 2

        middle_x = image_width / 2 - 0.5
        middle_y = image_height / 2 - 0.5

        for i in range(0, image_height):
            j_min = round(middle_x - math.sqrt(pow(radius - 0.01, 2) - pow(i - middle_y, 2)))
            j_max = round(middle_x + math.sqrt(pow(radius - 0.01, 2) - pow(i - middle_y, 2)))
            for j in range(j_min, j_max):
                red_pixel = image_data[i * image_width * 3 + 3 * j]
                green_pixel = image_data[i * image_width * 3 + 3 * j + 1]
                blue_pixel = image_data[i * image_width * 3 + 3 * j + 2]

                red_total += red_pixel
                green_total += green_pixel
                blue_total += blue_pixel
                total_pixels += 255

                self.add_dominant_color_to_dict(red_pixel, green_pixel, blue_pixel, color_dict)

        self.color_dict = color_dict
        self.raw_total = red_total, green_total, blue_total, total_pixels
        self.ready = True

    def add_dominant_color_to_dict(self, red_pixel, green_pixel, blue_pixel, color_dict):
        if red_pixel + green_pixel + blue_pixel < 160 and red_pixel < 80 and green_pixel < 80 and blue_pixel < 80:
            color_dict['black'] += 1
        elif red_pixel + green_pixel + blue_pixel > 600:
            color_dict['white'] += 1
        elif red_pixel > green_pixel + blue_pixel or (red_pixel + green_pixel + blue_pixel > 500 and
                red_pixel > green_pixel and red_pixel > blue_pixel):
            color_dict['red'] += 1
        elif green_pixel > red_pixel + blue_pixel or (red_pixel + green_pixel + blue_pixel > 500 and
                green_pixel > red_pixel and green_pixel > blue_pixel):
            color_dict['green'] += 1
        elif blue_pixel > green_pixel + red_pixel or (red_pixel + green_pixel + blue_pixel > 500 and
                blue_pixel > red_pixel and blue_pixel > green_pixel):
            color_dict['blue'] += 1
        else:
            color_dict['none'] += 1

    def get_image_raw_color(self):
        raw_total = self.raw_total
        red_total = raw_total[0]
        green_total = raw_total[1]
        blue_total = raw_total[2]
        max = raw_total[3]

        return int('{0:g}'.format(round((red_total / max) * 255, 0))), \
               int('{0:g}'.format(round((green_total / max) * 255, 0))), \
               int('{0:g}'.format(round((blue_total / max) * 255, 0)))

    def get_image_reflect(self):
        raw_total = self.raw_total
        if raw_total is None:
            return 0
        red_total = raw_total[0]
        green_total = raw_total[1]
        blue_total = raw_total[2]
        max = raw_total[3]
        return (red_total + green_total + blue_total) / (3 * max)
