""" tools to manipulate goal poses """


class GoalPose():
    """ pose of goal """

    source = None
    target = None
    goal_type = ''

    def __init__(self, source=None, target=None, goal_type=''):
        """ creates a goal pose

        :param target: target pose
        :type target: Pose
        :param source: source pose
        :type source: Pose
        """

        self.source = source
        self.target = target
        self.goal_type = goal_type

    def parameters(self):
        """ return parameters in order to launch funtions """
        return self.source, self.target, self.goal_type
