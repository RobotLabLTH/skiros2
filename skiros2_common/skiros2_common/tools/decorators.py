from functools import wraps


class PrettyMetaClass(type):
    """
    Meta class for adding a simple string representation to a class
    """
    def __repr__(cls):
        return cls.__name__

    def __str__(cls):
        return cls.__name__


class PrettyObject(object):
    """
    Base class for adding a simple string representation to a class instance
    """
    __metaclass__ = PrettyMetaClass

    def __repr__(self):
        return self.class_name

    def __str__(self):
        return self.class_name

    @property
    def class_name(self):
        """
        Return the name of the class in a readable way
        """
        return type(self).__name__


def abstractmethod(func):
    """
    Simple decorator for abstract methods.
    Raise an NotImplementedError if not overridden by derived classes.
    """
    @wraps(func)
    def decorator(self, *args, **kwargs):
        raise NotImplementedError("\"{}.{}\" is an abstract method and needs to be overridden in the derived class.".format(type(self).__name__, func.__name__))
    return decorator


def prevent(**attributes):
    """
    Decorator for abstract methods.
    Raise an AttributeError if an attribute does not exist or has a specified value.
    """
    def decorator(func):
        @wraps(func)
        def wrapped(self, *args, **kwargs):
            for attr, value in attributes.items():
                if not hasattr(self, attr) or getattr(self, attr) is value:
                    raise AttributeError("Attribute self.{} must not equal {}.".format(attr, value))
            return func(self, *args, **kwargs)
        return wrapped
    return decorator
