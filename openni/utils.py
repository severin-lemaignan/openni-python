import ctypes


class InitializationError(Exception):
    pass


class OpenNIError(Exception):
    def __init__(self, code, message, logfile):
        self.code = code
        self.logfile = logfile
        Exception.__init__(self, code, message, logfile)


class NiteError(Exception):
    def __init__(self, code):
        Exception.__init__(self, code)


def inherit_properties(struct, attrname):
    def deco(cls):
        for name, _ in struct._fields_:
            def getter(self, name=name):
                return getattr(getattr(self, attrname), name)

            def setter(self, value, name=name):
                return setattr(getattr(self, attrname), name, value)
            setattr(cls, name, property(getter, setter))
        return cls
    return deco


class ClosedHandleError(Exception):
    pass


class ClosedHandle(object):
    def __getattr__(self, name):
        raise ClosedHandleError("Invalid handle")

    def __bool__(self):
        return False
    __nonzero__ = __bool__


ClosedHandle = ClosedHandle()


class HandleObject(object):
    __slots__ = ["_handle"]

    def __init__(self, handle):
        self._handle = handle

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, t, v, tb):
        self.close()

    def __bool__(self):
        return hasattr(self, "_handle") and bool(self._handle)
    __nonzero__ = __bool__

    def close(self):
        if hasattr(self, "_handle") and self._handle:
            self._close()
            self._handle = ClosedHandle

    def _close(self):
        raise NotImplementedError()


def _py_to_ctype_obj(obj):
    size = None
    if isinstance(obj, (int, bool)):
        obj = ctypes.c_int(obj)
    elif isinstance(obj, float):
        obj = ctypes.c_float(obj)
    elif isinstance(obj, str):
        obj = ctypes.create_string_buffer(obj)
        size = len(obj)
    return obj, size


class CEnumMeta(type(ctypes.c_int)):
    def __new__(cls, name, bases, namespace):
        cls2 = type(ctypes.c_int).__new__(cls, name, bases, namespace)
        if namespace.get("__module__") != __name__:
            namespace["_values_"].clear()
            for name in namespace["_names_"].keys():
                if name.startswith("_"):
                    continue
                setattr(cls2, name, cls2(namespace[name]))
                namespace["_names_"][name] = namespace[name]
                namespace["_values_"][namespace[name]] = name
        return cls2


def with_meta(meta, base=object):
    return meta("NewBase", (base,), {"__module__": __name__})


class CEnum(with_meta(CEnumMeta, ctypes.c_int)):
    _names_ = {}
    _values_ = {}
    __slots__ = []

    def __repr__(self):
        name = self._values_.get(self.value)
        if name is None:
            return "%s(%r)" % (self.__class__.__name__, self.val)
        else:
            return "%s.%s" % (self.__class__.__name__, name)

    @classmethod
    def from_param(cls, obj):
        return int(obj)

    @classmethod
    def from_name(cls, name):
        return cls._names_[name]

    @classmethod
    def from_value(cls, val):
        return getattr(self, cls._values_[val])

    def __int__(self):
        return int(self.value)

    def __index__(self):
        return int(self)

    def __eq__(self, other):
        return int(self) == int(other)

    def __ne__(self, other):
        return int(self) != int(other)

    def __gt__(self, other):
        return int(self) > int(other)

    def __ge__(self, other):
        return int(self) >= int(other)

    def __lt__(self, other):
        return int(self) < int(other)

    def __le__(self, other):
        return int(self) <= int(other)

    def __hash__(self):
        return hash(int(self))


class DLLNotLoaded(Exception):
    pass


class UnloadedDLL(object):
    __slots__ = []

    def __bool__(self):
        return False
    __nonzero__ = __bool__

    def __call__(self, *args, **kwargs):
        raise DLLNotLoaded("DLL is not loaded")

    def __getattr__(self, name):
        raise DLLNotLoaded("DLL is not loaded")


UnloadedDLL = UnloadedDLL()
