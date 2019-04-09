#################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Bjarne Grossmann
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################################


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
            for attr, value in attributes.iteritems():
                if not hasattr(self, attr) or getattr(self, attr) is value:
                    raise AttributeError("Attribute self.{} must not equal {}.".format(attr, value))
            return func(self, *args, **kwargs)
        return wrapped
    return decorator
