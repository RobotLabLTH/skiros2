from rclpy.node import Node
from multiprocessing.dummy import Process


class DiscreteReasoner(object):
    from skiros2_world_model.ros import world_model_interface

    def __repr__(self):
        return self.__class__.__name__

    def init(self, wmi: world_model_interface, node: Node):
        """
        Set an interface to the world model and the ROS node
        """
        self._stop_requested = False
        self._thread = None
        self._wmi = wmi
        self.init_ros(node)

    def init_ros(self, node):
        """
        Set an interface to the ROS2 network
        """
        self._node = node

    def parse(self, element, action):
        """ Parse the action (add, remove, update) [element] """
        raise NotImplementedError("Not implemented in abstract class")

    @property
    def stopRequested(self):
        """ Returns True if stop execution flag has been raised and set the flag back to False """
        to_ret = self._stop_requested
        self._stop_requested = False
        return to_ret

    def stop(self):
        """ Raise the stop flag for the running reasoner """
        self._stop_requested = True

    def execute(self):
        self._thread = Process(target=self.run)
        self._thread.start()

    def run(self):
        """ Run the reasoner daemon on the world model """
        raise NotImplementedError("Not implemented in abstract class")

    def addProperties(self, element):
        """ Tag element with discrete reasoner id, then call onAddProperties """
        if not element.hasProperty("skiros:DiscreteReasoner", self.__class__.__name__):
            element.setProperty("skiros:DiscreteReasoner", self.__class__.__name__)
        self.onAddProperties(element)

    def onAddProperties(self, element):
        """ Add default reasoner properties to the element """
        raise NotImplementedError("Not implemented in abstract class")

    def removeProperties(self, element):
        """ Remove default reasoner properties from the element """
        if element.hasProperty("skiros:DiscreteReasoner", self.__class__.__name__):
            element.getProperty("skiros:DiscreteReasoner").removeValue(self.__class__.__name__)
        self.onRemoveProperties(element)

    def onRemoveProperties(self, element):
        """ Remove default reasoner properties from the element """
        raise NotImplementedError("Not implemented in abstract class")

    def hasData(self, element, get_code):
        """ Return data from the element in the format indicated in get_code """
        raise NotImplementedError("Not implemented in abstract class")

    def getData(self, element, get_code):
        """ Return data from the element in the format indicated in get_code """
        raise NotImplementedError("Not implemented in abstract class")

    def setData(self, element, data, set_code):
        """ Convert user data to reasoner data and store it into given element """
        raise NotImplementedError("Not implemented in abstract class")

    def getAssociatedRelations(self):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")

    def getAssociatedProperties(self):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")

    def getAssociatedData(self):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")

    def computeRelations(self, sub, obj, with_metrics=False):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")
