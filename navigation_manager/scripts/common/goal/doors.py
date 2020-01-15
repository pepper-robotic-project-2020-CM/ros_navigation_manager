""" Utilities to manipulate doors

You can use the `Door` class to manipulate doors waypoints
And the DoorDetector to detect door on the robot trajectory
"""

import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from robocup_msgs.msg import InterestPoint, InterestPoints

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Door:
    """ manipulate door waypoints """

    reverse = False
    interest_point = None

    def __init__(self, interest_point, reverse=False):
        """ Create a Door

        :param interest_point: interest point of door type
        :type: InterestPoint
        :param reverse: define if the door is in the other direction
        """
        self.reverse = reverse
        self.interest_point = interest_point

    def transpose(self, distance=0.5):
        """ transpose a waypoint and get translated pos

        :param dist: distance of the transposition in meters
        :type dist: int

        :return: tranposed pose
        :rtype: Pose
        """
        x = self.interest_point.pose.position.x
        y = self.interest_point.pose.position.y
        (_, _, th) = euler_from_quaternion(
            self.interest_point.pose.orientation
        )

        # translated taking the opposite of the door direction
        th = th if self.reverse else -th

        transposed_pose = Pose()
        transposed_pose.orientation = self.interest_point.pose.orientation
        transposed_pose.position.x = x + distance * math.cos(th)
        transposed_pose.position.y = y + distance * math.sin(th)
        return transposed_pose

    def on_path(self, path, distance_threshold=0.7):
        """ verify if the door is on the given path

        If the door interest point is under the distance_threshold,
        the door is considerd to be on the path

        The verification of the direction is to be added later

        :param path: path on which to verify if there is a door
        :type path: Pose
        :param distance_threshold: distance
        :type distance_threshold: int

        :return: list containing a Door if the door is on the path
                 or nothing if there is no door
        :rtype: list<Door>
        """
        cls = Door

        for pose in path.poses:
            path_x, path_y = pose.pose.position.x, pose.pose.position.y
            door_x = self.interest_point.pose.position.x
            door_y = self.interest_point.pose.position.y

            dist_to_door = math.sqrt(
                pow(path_y - door_y, 2) + pow(path_x - door_x, 2)
            )
            if dist_to_door > distance_threshold:
                # TODO: verify if in right direction
                # TODO: return reversed door if door in the other direction
                return [cls(self.interest_point, reverse=False)]

        return []


class DoorDetector:
    """ detection of doors on path """

    doors = []

    def __init__(
        self,
        doors_detected_callback,
        interest_points_topic="/interest_points",
        path_topic="/move_base/DWAPlannerROS/global_plan",
        distance_threshold=0.2
    ):
        """ create a door detector

        :param doors_callback: function to be called when door is detected
                               definition of the function is
                               (door: Door) -> None
        :param doors_topic: topic of doors
        :param path_topic: topic of path
        """

        self.doors_sub = rospy.Subscriber(
            interest_points_topic, InterestPoints, self.doors_callback
        )
        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback)

        self.door_detected_callback = doors_detected_callback
        self.distance_threshold = distance_threshold

    def doors_callback(self, msg):
        """ callback when door message is received """
        self.doors = [
            Door(interest_point)
            for interest_point in msg.interest_points
            if interest_point.type == "door_goal"
        ]

    def path_callback(self, msg):
        """ callback when path message is received """
        doors_on_path = self.on_path(msg)
        if doors_on_path:
            self.door_detected_callback(doors_on_path)

    def on_path(self, path):
        """ look if there are doors on path

        It returns a set with the doors on the path

        :param path: Path on which to search for doors
        :type path: Path

        :return: return a list containing the doors on the path
        :rtype: list<Door>
        """
        doors_on_path = []
        for door in self.doors:
            doors_on_path += door.on_path(path, self.distance_threshold)

        return doors_on_path

    def first(self, path):
        """ returns the first door

        :param path: Path on which to search for doors
        :type path: Path
        """

        doors = self.on_path(path)
        if doors:
            return doors[0]
        else:
            return None
