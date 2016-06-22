import cv2
import pcl
from primesense import openni2

import numpy as np

import re
import yaml
import pprint


class NiDevice:
    def __init__(self, uri, yamlfile):
        self.__uri = uri
        self.__yamlfile = yamlfile

    def initialize(self):
        self.__device = openni2.Device.open_file(self.__uri)

        if self.__device.is_file():
            self.__initialize_file_playback_control()

            m = re.search(r'.+\.\d+_(\d+)\.oni$', self.__uri)
            self.__serial = m.group(1)

            if len(self.__serial) > 0:
                self.__load_config()

        self.__color_stream = self.__create_and_start_stream(openni2.SENSOR_COLOR)
        self.__depth_stream = self.__create_and_start_stream(openni2.SENSOR_DEPTH)

        self.__frame_width = max(self.__color_stream.video_mode.resolutionX, self.__depth_stream.video_mode.resolutionX)
        self.__frame_height = max(self.__color_stream.video_mode.resolutionY, self.__depth_stream.video_mode.resolutionY)

    def update(self):
        color_image = self.__update_color_image()
        depth_image = self.__update_depth_image()

        points = []

        for x in range(0, self.__frame_width):
            for y in range(0, self.__frame_height):
                point = pcl.PointCloudXYZRGBA()
                point.z = depth_image[x, y]
                point.x = (x - self.__cx) * point.z / self.__fx
                point.x = (y - self.__cy) * point.z / self.__fy
                point.b = color_image[x, y][0]
                point.g = color_image[x, y][1]
                point.r = color_image[x, y][2]
                points.append(point)

        cloud = pcl.PointCloud()
        cloud.from_array(np.array(points))



    def __initialize_file_playback_control(self):
        self.__device.playback.set_speed(-1)
        self.__device.playback.set_repeat_enabled(False)

    def __load_config(self):
        assert len(self.__serial) > 0, 'Serial number is required.'
        prefix = 'kinect2_'

        with open(self.__yamlfile, 'r') as f:
            # Skip the first line "%YAML:1.0"
            f.readline()
            fs = yaml.load(f.read().replace('!!opencv-matrix', ''))

        yaml_camera_matrix = fs[prefix + self.__serial]['camera_matrix']
        camera_matrix = np.matrix(np.array(yaml_camera_matrix['data']))
        camera_matrix.shape = (yaml_camera_matrix['rows'], yaml_camera_matrix['cols'])

        yaml_rot = fs[prefix + self.__serial]['rotation']
        rot = np.matrix(np.array(yaml_rot['data']))
        rot.shape = (yaml_rot['rows'], yaml_rot['cols'])

        yaml_tr = fs[prefix + self.__serial]['translation']
        tr = np.matrix(np.array(yaml_tr['data']))
        tr.shape = (yaml_tr['rows'], yaml_tr['cols'])

        tmp_matrix = np.matrix(np.identity(4))
        tmp_matrix[0:3, 0:3] = rot
        tmp_matrix[0:3, 3] = tr

        self.__calib_matrix = np.linalg.inv(tmp_matrix)
        self.__fx = camera_matrix[0, 0]
        self.__fy = camera_matrix[1, 1]
        self.__cx = camera_matrix[0, 2]
        self.__cy = camera_matrix[1, 2]

    def __create_and_start_stream(self, seonsor_type):
        if not self.__device.has_sensor(seonsor_type):
            return None
        stream = openni2.VideoStream(self.__device, seonsor_type)
        stream.start()
        return stream

    def __update_color_image(self):
        color_image = np.matrix(np.array(self.__color_stream.read_frame().get_buffer_as_uint8()))
        color_image.shape = (self.__color_stream.video_mode.resolutionX, self.__color_stream.video_mode.resolutionY)
        return cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

    def __update_depth_image(self):
        depth_image = np.matrix(np.array(self.__depth_stream.read_frame().get_buffer_as_uint16()))
        depth_image.shape = (self.__depth_stream.video_mode.resolutionX, self.__depth_stream.video_mode.resolutionY)
        return depth_image[0:self.__frame_width, 0:self.__frame_height]
