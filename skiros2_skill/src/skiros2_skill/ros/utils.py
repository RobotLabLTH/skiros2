import skiros2_msgs.msg as msgs
import skiros2_common.core.params as params
import skiros2_common.ros.utils as utils

class SkillHolder(object):
    __slots__ = ['manager', 'type', 'name', 'ph']
    def __init__(self, manager, type_in, name, params_in=None):
        self.manager = manager
        self.name = name
        self.type = type_in
        self.ph = params.ParamHandler()
        if params_in:
            self.ph.reset(params_in)
        
    def __str__(self):
        return "{}:{} [{}]".format(self.type, self.name, self.ph.printState())
        
    def toMsg(self):
        msg = msgs.ResourceDescription()
        msg.type = self.type
        msg.name = self.name
        msg.params = utils.serializeParamMap(self.ph.getParamMap())
        return msg