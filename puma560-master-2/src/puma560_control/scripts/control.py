#!/usr/bin/env python3

# Importing required libraries
import sympy    # For symbolic mathematics
import rospy    # For ROS python interface
import numpy as np  # For numerical operations
import math as m    # For mathematical functions like atan2, sqrt
import sys     # For system-specific parameters and functions

# Importing specific classes and functions from ROS libraries
from sympy import cos, sin, pi
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker

# Class to control PUMA 560 robot
class puma560_control:
    def __init__(self):
        # Initializing ROS node
        rospy.init_node("puma560_control", anonymous=True)

        # Publisher to show visualization markers in RViz
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

        # Subscriber to get the positions of all models in Gazebo
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)

        # Publishers to send joint commands to PUMA 560 robot
        self.th1_pub = rospy.Publisher("/puma560_description/j1_pc/command", Float64, queue_size=1)
        self.th2_pub = rospy.Publisher("/puma560_description/j2_pc/command", Float64, queue_size=1)
        self.th3_pub = rospy.Publisher("/puma560_description/j3_pc/command", Float64, queue_size=1)
        self.th4_pub = rospy.Publisher("/puma560_description/j4_pc/command", Float64, queue_size=1)
        self.th5_pub = rospy.Publisher("/puma560_description/j5_pc/command", Float64, queue_size=1)

        # Denavit-Hartenberg Parameters (a and d values for robot links)
        self.a = [0, 0, 0.4318, 0.0203, 0, 0]
        self.d = [0, 0, 0.1500, 0.4318, 0, 0]

        # Perform symbolic forward kinematics calculations
        self.Toriginefector, self.b, self.T36 = self.sym_fwd_kinematics()

        # Spawn a red box (object) in Gazebo at given position and size
        self.spawn_box(name="red_box", position=[0.5, 0, 0.05], size=[0.1, 0.1, 0.1])

    # Callback function for model states to track object position
    def model_callback(self, msg):
        for name in ["red_box"]:
            try:
                # Get index of the object
                idx = msg.name.index(name)
                pose = msg.pose[idx]

                # Create a marker to show the object in RViz
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "spawned_objects"
                marker.id = 0
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose = pose
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 1.0  # Red color
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # Publish the marker
                self.marker_pub.publish(marker)

                # Call inverse kinematics to move robot to object position
                self.inv_kinematics(pose.position.x, pose.position.y, pose.position.z + 0.05)

            except ValueError:
                rospy.logwarn_once(f"{name} not found in model_states")

    # Function to spawn box in Gazebo
    def spawn_box(self, name="red_box", position=[0.5, 0, 0.05], size=[0.1, 0.1, 0.1]):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

            # Define SDF (Simulation Description Format) of box
            sdf_box = f"""<?xml version='1.0'?>
<sdf version='1.6'>
<model name='{name}'>
    <static>true</static>
    <link name='link'>
    <pose>0 0 {size[2]/2} 0 0 0</pose>
    <collision name='collision'>
        <geometry>
        <box><size>{size[0]} {size[1]} {size[2]}</size></box>
        </geometry>
    </collision>
    <visual name='visual'>
        <geometry>
        <box><size>{size[0]} {size[1]} {size[2]}</size></box>
        </geometry>
        <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
        </material>
    </visual>
    </link>
</model>
</sdf>"""

            # Define pose of box
            pose = Pose(position=Point(*position), orientation=Quaternion(0, 0, 0, 1))

            # Call Gazebo service to spawn box
            spawn_model(model_name=name, model_xml=sdf_box, robot_namespace="/", initial_pose=pose, reference_frame="world")
            rospy.loginfo(f"Spawned model: {name}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed: {e}")

    # Function to calculate symbolic forward kinematics
    def sym_fwd_kinematics(self):
        i = range(6)
        T = []  # List to store transformation matrices
        Toriginefector = 1  # Initial transformation matrix
        thi, alj, aj, di = sympy.symbols("thi alj aj di")

        # General transformation matrix using DH parameters
        Tij = sympy.Matrix([[cos(thi), -sin(thi), 0, aj],
                            [sin(thi)*cos(alj), cos(thi)*cos(alj), -sin(alj), -sin(alj)*di],
                            [sin(thi)*sin(alj), cos(thi)*sin(alj), cos(alj), cos(alj)*di],
                            [0, 0, 0, 1]])

        th1, th2, th3, th4, th5, th6, a2, a3, d3, d4 = sympy.symbols("th1 th2 th3 th4 th5 th6 a2 a3 d3 d4")
        al = [0, -pi/2, 0, -pi/2, pi/2, -pi/2]
        a = [0, 0, a2, a3, 0, 0]
        d = [0, 0, d3, d4, 0, 0]
        th = [th1, th2, th3, th4, th5, th6]

        # Calculate transformation matrix for each joint
        for j in i:
            Tx = Tij.subs([(thi, th[j]), (alj, al[j]), (aj, a[j]), (di, d[j])])
            T.append(Tx)

        # Multiply all matrices to get final transformation matrix
        for j in i:
            Toriginefector *= T[j]

        Toriginefector = sympy.simplify(Toriginefector)

        # Calculate inverse of T03 matrix
        T03inv = sympy.simplify((T[0]*T[1]*T[2]).inv())

        Tsym = sympy.MatrixSymbol('T', 4, 4)

        b = T03inv * Tsym
        T36 = sympy.simplify(T03inv * Toriginefector)

        return Toriginefector, b, T36

    # Function to publish joint angles to the robot
    def publisher(self, th1, th2, th3, th4, th5):
        self.th1_pub.publish(Float64(th1))
        self.th2_pub.publish(Float64(th2))
        self.th3_pub.publish(Float64(th3))
        self.th4_pub.publish(Float64(th4))
        self.th5_pub.publish(Float64(th5))

    # Inverse kinematics calculation to find joint angles
    def inv_kinematics(self, Px, Py, Pz):
        a2, a3, d3, d4 = self.a[2], self.a[3], self.d[2], self.d[3]

        # Calculate joint angles based on the given end-effector position
        th1 = m.atan2(Py, Px) - m.atan2(d3, m.sqrt(Px**2 + Py**2 - d3**2))
        k = (Px**2 + Py**2 + Pz**2 - a2**2 - a3**2 - d3**2 - d4**2)/(2*a2)
        th3 = m.atan2(a3, d4) - m.atan2(k, m.sqrt(a3**2 + d4**2 - k**2))
        th2 = m.atan2((Px*m.cos(th1) + Py*m.sin(th1))*(a2*m.sin(th3) - d4) - Pz*(a3 + a2*m.cos(th3)),
                      (a3 + a2*m.cos(th3))*(Px*m.cos(th1) + Py*m.sin(th1)) + (a2*m.sin(th3) - d4)*Pz) - th3
       
        # Publish the calculated joint angles
        self.publisher(th1, th2, th3)

# Main function to run the code
if __name__ == "__main__":
    try:
        p560_c = puma560_control()
        rospy.spin()  # Keeps the node running
    except ROSInterruptException:
        sys.exit()
