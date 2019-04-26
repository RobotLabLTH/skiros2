class DictDiff(object):
    """
    Calculate the difference between two dictionaries as:
    (1) items added
    (2) items removed
    (3) keys same in both but changed values
    (4) keys same in both and unchanged values
    """

    def __init__(self, current_dict, past_dict):
        self.current_dict, self.past_dict = current_dict, past_dict
        self.set_current, self.set_past = set(current_dict.keys()), set(past_dict.keys())
        self.intersect = self.set_current.intersection(self.set_past)

    @property
    def added(self):
        return self.set_current - self.intersect

    @property
    def removed(self):
        return self.set_past - self.intersect

    @property
    def changed(self):
        return [o for o in self.intersect if self.past_dict[o] != self.current_dict[o]]

    @property
    def changed_values(self):
        return {o: self.current_dict[o] for o in self.intersect if self.past_dict[o] != self.current_dict[o]}

    @property
    def unchanged(self):
        return [o for o in self.intersect if self.past_dict[o] == self.current_dict[o]]
