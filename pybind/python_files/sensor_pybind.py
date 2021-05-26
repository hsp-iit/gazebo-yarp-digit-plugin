import os
import logging
import warnings
import collections
from dataclasses import dataclass
import hydra
import cv2
import numpy as np
import trimesh
from urdfpy import URDF
from tacto import Renderer
import time
import math


"""
This import sys was a necessary workaround, since when we are using
a library with embedded python sys.argv is not defined

"""
import sys

if not hasattr(sys, 'argv'):
    sys.argv  = ['']


#function to get the path of the configuration file to be passed to the renderer
def _get_default_config(filename):                                      
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), filename)#__file__

def get_digit_config_path():
    return _get_default_config("../../tacto/tacto/config_digit.yml")


#At the moment, we are taking into account that we have just one sensor
@dataclass
class Sensor:
    def __init__(self):
        """
        :param width: scalar
        :param height: scalar
        :param background: image
        :param visualize_gui: Bool
        :param show_depth: Bool
        :param config_path:
        :param cid: Int
        """
        self.cid = 0
        self.width=120
        self.height=160
        bg = cv2.imread("../../tacto/examples/conf/bg_digit_240_320.jpg") #
        print(bg)
        self.renderer = Renderer(self.width, self.height, bg, get_digit_config_path())
        self.visualize_gui = True
        self.show_depth = True
        self.zrange = 0.002
        self.object_poses = {}
        self.normal_forces = {}
        self._static = None
        self.position= 0

    #method to call when we have to add an object to the scene
    def add_object(self, mesh, obj_id, position=[0.0,0.0,0.0], orientation = [0.0,0.0,0.0], globalScaling=1.0): #global scaling is not useful anymore

        # Load the mesh
        obj_trimesh = trimesh.load(mesh)


        # Set mesh color to default (remove texture)
        obj_trimesh.visual = trimesh.visual.ColorVisuals()


        #the renderer expects to receive a certain name
        #we are keeping the same notation as before
        #it is not necessary
        obj_name = "{}_{}".format(2, -1)


        self.object_poses[obj_name]=position, orientation 


        # Add object in pyrender
        self.renderer.add_object(
            obj_trimesh,
            obj_name,
            position=self.object_poses[obj_name][0],  # 
            orientation=self.object_poses[obj_name][1],  # 
        )

   

    @property
    def static(self):
        if self._static is None:
            colors, _ = self.renderer.render(noise=False)
            depths = [np.zeros_like(d0) for d0 in self.renderer.depth0]
            self._static = (colors, depths)

        return self._static

    def _render_static(self):
        colors, depths = self.static
        colors = [self.renderer._add_noise(color) for color in colors]
        return colors, depths

    def render(self):

        colors = []
        depths = []

       
        cam_name = "cam0" 

        # get the contact normal forces
        # At the moment, we are forcing a certain value since we are have no physical environment

        self.normal_forces["2_-1"] = 10.0
        if self.normal_forces["2_-1"] >0.0:
            position, orientation = [0.0,0.0,0.0],[0.0, -1.5707963267948966,0.0] #self.cameras[cam_name].get_pose()
            self.renderer.update_camera_pose(position, orientation)
            self.position+=0.2
            self.object_poses["2_-1"][0][2]+=math.sin(self.position)/400#we are just adding a sinusoid in order to see how the image given by the sensor changes
            color, depth = self.renderer.render(self.object_poses, self.normal_forces)

            # Remove the depth from curved gel
            for j in range(len(depth)):
                depth[j] = self.renderer.depth0[j] - depth[j]
        else:
            color, depth = self._render_static()

        colors += color
        depths += depth

        return colors, depths

    def _depth_to_color(self, depth):
        gray = (np.clip(depth / self.zrange, 0, 1) * 255).astype(np.uint8)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    """
    def updateGUI(self, colors, depths):
        """"""
        Update images for visualization
        
        """"""
        if not self.visualize_gui:
            return

        # concatenate colors horizontally (axis=1)
        color = np.concatenate(colors, axis=1)

        if self.show_depth:
            # concatenate depths horizontally (axis=1)
            depth = np.concatenate(list(map(self._depth_to_color, depths)), axis=1)

            # concatenate the resulting two images vertically (axis=0)
            color_n_depth = np.concatenate([color, depth], axis=0)

            cv2.imshow("color and depth", color_n_depth)
            cv2.imwrite("color and depthx25.jpg", color_n_depth)
        else:
            cv2.imshow("color", color)
        cv2.waitKey(1)"""
    def updateGUI(self, colors, depths):
        """
        Update images for visualization
        
        """
        if not self.visualize_gui:
            return

        # concatenate colors horizontally (axis=1)
        color = np.concatenate(colors, axis=1)

        if self.show_depth:
            # concatenate depths horizontally (axis=1)
            depth = np.concatenate(list(map(self._depth_to_color, depths)), axis=1)

            # concatenate the resulting two images vertically (axis=0)
            color_n_depth = np.concatenate([color, depth], axis=0)     

            #cv2.imshow("color and depth", color_n_depth)
            #cv2.imwrite("color and depthx25.jpg", color_n_depth)
            #cv2.waitKey(1)
            return color                            # at the moment, we would like to see just the color image
        else:
            #cv2.imshow("color", color)
            #cv2.imwrite("color and depthx25.jpg", color)
            #cv2.waitKey(1)
            return color
        

    def get_image(self):                            #method to call when we want to save and visualize an image
        colors, depth=self.render()
        return self.updateGUI(colors, depth)
