from std_msgs.msg import Empty
from skiros2_msgs.msg import ManagerDescription
import rospy


class DiscoverableNode(object):
    """
    A node that publish its name on /description whenever a msg on /discovery is delivered
    """

    def init_discovery(self, namespace, node_id):
        self._ns = namespace
        self._node_id = node_id
        self._sub_discovery = rospy.Subscriber('/{}/discovery'.format(self._ns), Empty, self._on_discovery)
        self._pub_description = rospy.Publisher('/{}/description'.format(self._ns), ManagerDescription, queue_size=10)
        rospy.sleep(0.5)  # Give time to ROS connection to initialize
        self.publish_description()

    def publish_description(self, state=ManagerDescription.ACTIVE):
        self._pub_description.publish(ManagerDescription(self._node_id, state))

    def shutdown_discovery(self):
        self._sub_discovery.unregister()
        self.publish_description(ManagerDescription.INACTIVE)
        self._pub_description.unregister()

    def _on_discovery(self, msg):
        self.publish_description(ManagerDescription.ACTIVE)


class DiscoveryInterface(object):
    """
    Interface to discoverable nodes
    """

    def init_discovery(self, namespace, callback):
        self._ns = namespace
        self._pub_discovery = rospy.Publisher('/{}/discovery'.format(self._ns), Empty, queue_size=10)
        self._sub_description = rospy.Subscriber('/{}/description'.format(self._ns), ManagerDescription, callback, queue_size=10)
        rospy.Timer(rospy.Duration(2), self.discover)

    def discover(self, event=None):
        self._pub_discovery.publish(Empty())
