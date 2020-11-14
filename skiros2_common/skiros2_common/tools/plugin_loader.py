import importlib
import pkgutil
import sys
import inspect

import re
import skiros2_common.tools.logger as log


class PluginLoader(object):

    @classmethod
    def __import_plugins(self, package, base):
        results = []

        if isinstance(package, str):
            package = importlib.import_module(package)

        for loader, modname, is_pkg in pkgutil.walk_packages(path=package.__path__,
                                                             prefix=package.__name__ + '.',
                                                             onerror=lambda x: None):
            if is_pkg:
                continue

            if modname not in sys.modules:
                try:
                    importlib.import_module(modname)
                except ImportError as e:
                    log.warn("[ImportError]", "Module {} not loaded, some dependencies are missing: {}".format(modname, e))
                    continue

            module = sys.modules[modname]

            def pred(member): return inspect.isclass(member) and \
                member.__module__ == modname and \
                issubclass(member, base) and \
                not member.__subclasses__()

            classes = dict(inspect.getmembers(module, pred))
            results.extend(classes.values())

        results = [c for c in results if not c.__subclasses__()]

        return results

    @classmethod
    def signature(self, plugin):
        args = inspect.getargspec(plugin.__init__)
        names = args[0]
        defaults = args[-1] if args[-1] is not None else []
        req = names[1:len(names) - len(defaults)]
        opt = dict(zip(names[len(names) - len(defaults):], defaults))
        return (names[1:], req, opt)

    @classmethod
    def split(self, plugin):
        if inspect.isclass(plugin):
            name = plugin.__name__
        else:
            name = type(plugin).__name__
        return plugin.__module__.split(".") + [name]

    @classmethod
    def match(self, plugin, desc=None):
        clazz = self.split(plugin)[::-1]
        for a, b in zip(clazz, desc):
            if b and not re.match("^" + b + "$", a):
                return False
        return True

    @classmethod
    def instance(self, plugin, args_dict):
        names, req, opt = self.signature(plugin)
        log.info(self.__class__.__name__, "Instantiating " + str(plugin) + " with arguments " +
                 str(args_dict) + "  || REQUIRED: " + str(req) + " OPTIONAL: " + str(opt.keys()))
        p = None
        try:
            p = plugin(**args_dict)
        except Exception as e:
            log.error(self.__class__.__name__, "  ERROR while instantiating: " + str(e))
        return p

    def __init__(self):
        self._plugins = list()

    def __iter__(self):
        if self._plugins:
            return iter(self._plugins)
        else:
            return iter([])

    def _filter(self, desc):
        return [p for p in self._plugins if self.match(p, desc)]

    def _exclude(self, desc):
        return [p for p in self._plugins if not self.match(p, desc)]

    def load(self, folder, base_class):
        self._plugins += self.__import_plugins(folder, base_class)
        if self.size() == 0:
            log.warn(self.__class__.__name__,
                     "No instances of {} found in {}! Check your package configuration.".format(base_class, folder))

    def size(self):
        return len(self._plugins)

    def create(self, args_dict={}):
        instances = []
        for p in self._plugins:
            names, req, opt = self.signature(p)
            args = opt.copy()
            args.update({k: args_dict[k] for k in req if k in args_dict.keys()})

            instance = self.instance(p, args)
            if instance is not None:
                instances.append(instance)

        return instances

    def getPluginByName(self, name):
        p = self._filter([name])
        if len(p) == 0:
            raise Exception("No plugin with name " + str(name) + " found!")
        elif len(p) > 1:
            log.warn(self.__class__.__name__, "WARNING: Multiple plugins with name " + str(name) + " found!\n" + str(p))
        return p[0]

    def list(self):
        for p in self._plugins:
            print(self.split(p))
        return [p.__name__ for p in self._plugins]
