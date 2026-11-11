import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import yaml
import os
from datetime import datetime

class YamlCoord(Node):