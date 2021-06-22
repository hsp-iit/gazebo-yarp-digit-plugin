import cv2
import numpy as np
import os
import trimesh
from tacto import Renderer
import sys


class Sensor:

    def __init__(self, background_path, configuration_path):

        # Check on the configuration file
        if not os.path.isfile(configuration_path):
            sys.exit("The configuration file " + configuration_path + " does not exist or it is not a valid file.")


        # Check on the background image
        bg = cv2.imread(background_path)
        if bg is None:
            sys.exit("The background image " + background_path + " does not exist or it is not a valid image.")


        # Camera parameters
        self.width = 240
        self.height = 320


        # Instantiate the renderer
        self.renderer = Renderer(self.width, self.height, bg, configuration_path)


        # Storage for object pose and forces
        self.object_pose = {}
        self.object_force = {}

        # Storage for sensor output in a static setting
        self.static_rgb = None


        # Store the object name
        self.object_name = None


    # Add new object to the simulation
    def add_object(self, mesh, object_name, position = [0.0,0.0,0.0], orientation = [0.0,0.0,0.0]):

        # Load the mesh
        obj_trimesh = trimesh.load(mesh)


        # Set mesh color to default (remove texture)
        obj_trimesh.visual = trimesh.visual.ColorVisuals()

        # Store the object name
        self.object_name = object_name


        # Add object in pyrender
        self.renderer.add_object(obj_trimesh, self.object_name, position = position, orientation = orientation)



    # Render the output of the sensor
    def render(self, object_position, object_orientation,  sensor_position, sensor_orientation, force):

        self.object_force[self.object_name] = force


        if self.object_force[self.object_name] > 0.0:

            # Update camera pose
            self.renderer.update_camera_pose(sensor_position, sensor_orientation)


            # Position and orientation of the object
            self.object_pose[self.object_name] = object_position, object_orientation


            # This function return RGB and depth matrices
            rgb, _ = self.renderer.render(self.object_pose, self.object_force)

        else:

            if self.static_rgb is None:

                sel.static_rgb, _ = self.renderer.render(noise = False)


            rgb = [self.renderer._add_noise(color) for color in self.static_rgb]


        rgb = np.concatenate(rgb, axis = 1)


        return rgb