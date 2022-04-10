import rospy


class Parameters:
    def __init__(self):
        self.parameter_set = False
        self.keywords_map = []
        self.states_map = dict()

    def initParameters(self):
        self.keywords_map = rospy.get_param('keywords_map')
        states = rospy.get_param('states_map')
        for st in states:
            self.states_map[st['name']] = st['key']

        self.speech_service = rospy.get_param('speech_service')
        self.listening_service = rospy.get_param('listening_service')
        self.usr_cmd_topic = rospy.get_param('usr_cmd_topic')
        self.ctrl_topic = rospy.get_param('ctrl_topic')
        self.force_topic = rospy.get_param('force_topic')
        self.encoder_topic = rospy.get_param('encoder_topic')
        self.reset_path_service = rospy.get_param('reset_path_service')
        self.planner_arrow_topic = rospy.get_param('planner_arrow_topic')
        self.global_map_topic = rospy.get_param('global_map_topic')
        self.planned_path_topic = rospy.get_param('planned_path_topic')

        self.model_path = rospy.get_param("model_path")
        self.map_file = rospy.get_param("map_file")

        self.parameter_set = True
