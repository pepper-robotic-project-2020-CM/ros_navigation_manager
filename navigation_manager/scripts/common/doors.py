""" Utilities to manipulate doors

You can use the `Door` class to manipulate doors waypoints
And the DoorDetector to detect door on the robot trajectory
"""

import math
import rospy
from nav_msgs.msg import Path
from robocup_msgs.msg import InterestPoint, InterestPoints


class Door():
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
        """
        pass

    def on_path(self, path, distance_treshold=0.2):
        """ verify if the door is on the given path

        If the door interest point is under the distance_treshold,
        the door is considerd to be on the path

        The verification of the direction is to be added later

        :param path: path on which to verify if there is a door
        :param distance_treshold: distance

        :return: set containing a Door if the door is on the path
                 or nothing if there is no door
        :rtype: set<Door>
        """
        cls = type(self)

        for pose in path:
            path_x, path_y = pose.pose.position.x, pose.pose.position.y
            door_x, door_y = self.interest_point.pose.position.x, self.interest_point.pose.position.y

            dist_to_door = math.sqrt(pow(path_y - door_y, 2),
                                     pow(path_x - door_x, 2))
            if dist_to_door > distance_treshold:
                # TODO: verify if in right direction
                # TODO: return reversed door if door in the other direction
                return set(cls(self.interest_point, reverse=False))

        return set()


class DoorDetector():
    """ detection of doors on path """

    doors = []

    def __init__(self,
                 doors_topic='',
                 path_topic='/move_base/DWAPlannerROS/global_plan'):
        """ create a door detector

        :param doors: topic of doors
        """

        # TODO: initialize door subscription topic
            # sub TODO: broadcast interest points from map_manager node
        self.doors_sub = rospy.Subscriber(doors_topic,
                                          InterestPoints,
                                          self.doors_callback)
        self.path_sub = rospy.Subscriber(path_topic,
                                         Path,
                                         self.path_callback)

    def doors_callback(self, msg):
        pass

    def path_callback(self, msg):
        doors_on_path = self.on_path(msg)
        if doors_on_path:
            # TODO: launch new dest ()
            # NOTE: it's not a good idea to add new dest here
            # It should be added in a objective pile
            pass

    def on_path(self, path):
        """ return doors on path

        :param path: Path on which to search for doors
        :type path: Path

        :return: return the doors on the path
        :rtype: list<Door>
        """
        doors_on_path = set()
        for door in self.doors:
            doors_on_path |= door.on_path(path)

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
