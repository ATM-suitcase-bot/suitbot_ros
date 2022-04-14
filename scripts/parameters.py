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
        self.voltage_topic = rospy.get_param('voltage_topic')

        self.reset_path_service = rospy.get_param('reset_path_service')
        self.planner_arrow_topic = rospy.get_param('planner_arrow_topic')
        self.global_map_topic = rospy.get_param('global_map_topic')
        self.planned_path_topic = rospy.get_param('planned_path_topic')

        self.lidar_original_topic = rospy.get_param('lidar_original_topic')
        self.lidar_sync_topic = rospy.get_param('lidar_sync_topic')

        self.model_path = rospy.get_param("model_path")
        self.map_file = rospy.get_param("map_file")

        self.manual_control = rospy.get_param("manual_control")
        self.init_x = rospy.get_param("init_x")
        self.init_y = rospy.get_param("init_y")
        self.init_theta = rospy.get_param("init_theta")

        self.debug_odometry = rospy.get_param("debug_odometry")
        self.debug_time = rospy.get_param("debug_time")
        self.debug_linear = rospy.get_param("debug_linear")
        self.debug_angular = rospy.get_param("debug_angular")

        self.use_audio = rospy.get_param("use_audio")

        self.parameter_set = True
