from std_msgs.msg import Empty, String
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node


class DiscoverableNode(Node):
    """
    @brief A node that publish its name on /description whenever a msg on /discovery is delivered
    """

    def init_discovery(self, namespace):
        self._ns = namespace
        self._sub_discovery = self.create_subscription(
            Empty, '/{}/discovery'.format(self._ns), self._on_discovery)
        self._pub_description = self.create_publisher(String, '/{}/description'.format(self._ns), 10)
        Duration(nanoseconds=5 * (10**8)).sleep()  # Give time to ROS connection to initialize
        self.publish_description()

    def publish_description(self):
        self._pub_description.publish(String(self.get_name()))

    def _on_discovery(self, msg):
        self.publish_description()


class DiscoveryInterface(object):
    """
    @brief Client interface to discoverable nodes
    """

    def init_discovery(self, node, namespace, on_active, on_inactive, discovery_period=1.0):
        """
        @brief Registers discovery callbacks
        @param namespace the ROS namespace for the discovery service
        @param on_active pointer to function expecting as input (string name). Called when a node gets active
        @param on_inactive pointer to function expecting as input (string name). Called when a node gets inactive
        """
        self._node = node
        self._ns = namespace
        self._active_nodes = dict()
        self.on_active = on_active
        self.on_inactive = on_inactive
        self._pub_discovery = self._node.create_publisher(Empty, '/{}/discovery'.format(self._ns), 10)
        self._sub_description = self._node.create_subscription(
            String, '/{}/description'.format(self._ns), self._description_cb, 10)
        self._node.create_timer(discovery_period, self.discover)
        self._active_timeout = discovery_period * 2  # Period before considering a node inactive

    def discover(self, event=None):
        self._pub_discovery.publish(Empty())
        for n, t in dict(self._active_nodes).items():
            if Time.now() - t > self._active_timeout:
                del self._active_nodes[n]
                self.on_inactive(n)  # Node inactive

    def _description_cb(self, msg):
        if msg.data not in self._active_nodes:
            self.on_active(msg.data)  # Node active
        self._active_nodes[msg.data] = Time.now()
