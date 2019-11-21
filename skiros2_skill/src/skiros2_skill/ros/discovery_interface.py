from std_msgs.msg import Empty, String
import rospy


class DiscoverableNode(object):
    """
    @brief A node that publish its name on /description whenever a msg on /discovery is delivered
    """
    def init_discovery(self, namespace, node_id):
        self._ns = namespace
        self._node_id = node_id
        self._sub_discovery = rospy.Subscriber('/{}/discovery'.format(self._ns), Empty, self._on_discovery)
        self._pub_description = rospy.Publisher('/{}/description'.format(self._ns), String, queue_size=10)
        rospy.sleep(0.5)  # Give time to ROS connection to initialize
        self.publish_description()

    def publish_description(self):
        self._pub_description.publish(String(self._node_id))

    def _on_discovery(self, msg):
        self.publish_description()


class DiscoveryInterface(object):
    """
    @brief Client interface to discoverable nodes
    """
    def init_discovery(self, namespace, on_active, on_inactive, discovery_period=rospy.Duration(1)):
        """
        @brief Registers discovery callbacks
        @param namespace the ROS namespace for the discovery service
        @param on_active pointer to function expecting as input (string name). Called when a node gets active
        @param on_inactive pointer to function expecting as input (string name). Called when a node gets inactive
        """
        self._ns = namespace
        self._active_nodes = dict()
        self.on_active = on_active
        self.on_inactive = on_inactive
        self._pub_discovery = rospy.Publisher('/{}/discovery'.format(self._ns), Empty, queue_size=10)
        self._sub_description = rospy.Subscriber('/{}/description'.format(self._ns), String, self._description_cb, queue_size=10)
        rospy.Timer(discovery_period, self.discover)
        self._active_timeout = discovery_period*2 #Period before considering a node inactive

    def discover(self, event=None):
        self._pub_discovery.publish(Empty())
        for n, t in dict(self._active_nodes).iteritems():
            if rospy.Time.now()-t>self._active_timeout:
                del self._active_nodes[n]
                self.on_inactive(n) #Node inactive

    def _description_cb(self, msg):
        if msg.data not in self._active_nodes:
            self.on_active(msg.data) #Node active
        self._active_nodes[msg.data] = rospy.Time.now()
