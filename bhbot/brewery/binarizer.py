# encoding: utf-8


import copy




class AbstractItem:

    C_TYPE = None
    DESCRIPTION = None


    def __init__(self, default_value, description = None):
        self.default_value = copy.deepcopy(default_value)
        if description is None:
            description = self.DESCRIPTION
        else:
            self.description = description


    def serialize(self, value, buf):
        buf.append(value)


    def deserialize(self, iterator):
        value = next(iterator)
        return value


    def to_dump(self, value):
        return str(value)


    def to_dump_as_text(self, value):
        return self.to_dump(value)


    def from_dump(self, value):
        return value




class Int8(AbstractItem):

    C_TYPE = 'b'
    DESCRIPTION = "1 byte signed integer (char)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class UInt8(AbstractItem):

    C_TYPE = 'B'
    DESCRIPTION = "1 byte unsigned integer (unsigned char)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class Int16(AbstractItem):

    C_TYPE = 'h'
    DESCRIPTION = "2 bytes signed integer (short)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class UInt16(AbstractItem):

    C_TYPE = 'H'
    DESCRIPTION = "2 bytes unsigned integer (unsigned short)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class Int32(AbstractItem):

    C_TYPE = 'i'
    DESCRIPTION = "4 bytes signed integer (int)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class UInt32(AbstractItem):

    C_TYPE = 'I'
    DESCRIPTION = "4 bytes unsigned integer (unsigned int)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class Int64(AbstractItem):

    C_TYPE = 'q'
    DESCRIPTION = "8 bytes signed integer (long long)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class UInt64(AbstractItem):

    C_TYPE = 'Q'
    DESCRIPTION = "8 bytes unsigned integer (unsigned long long)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)




class Float(AbstractItem):

    C_TYPE = 'f'
    DESCRIPTION = "4 bytes real (float)"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)


    def to_dump(self, value):
        return "{:0.4f}".format(value) if value else str(value)


    def from_dump(self, value):
        return float(value)




class Double(Float):

    C_TYPE = 'd'
    DESCRIPTION = "8 bytes real (double)"

    def __init__(self, default_value, description = None):
        Float.__init__(self, default_value, description)




class FloatRadian(Float):

    DESCRIPTION = "4 bytes real (float) radians"

    def __init__(self, default_value, description = None):
        Float.__init__(self, default_value, description)




class DoubleRadian(FloatRadian):

    C_TYPE = 'd'
    DESCRIPTION = "8 bytes real (double) radians"

    def __init__(self, default_value, description = None):
        FloatRadian.__init__(self, default_value, description)




class Bool(UInt8):

    DESCRIPTION = "8 bytes boolean value (unsigned char)"

    def __init__(self, default_value, description = None):
        UInt8.__init__(self, default_value, description)


    def serialize(self, value, buf):
        if value:
            buf.append(1)
        else:
            buf.append(0)


    def deserialize(self, iterator):
        value = next(iterator)
        return value != 0




class Enum8(Int8):

    DESCRIPTION = "8 bytes enum value (char)"

    def __init__(self, enum, default_value):
        self.enum = enum
        Int8.__init__(self, default_value, self.enum.description)


    def to_dump(self, value):
        try:
            v = self.enum.lookup_by_value[value]
        except:
            v= value
        return v


    def from_dump(self, value):
        return value




class UEnum8(Enum8):

    C_TYPE = 'B'
    DESCRIPTION = "8 bytes enum value (unsigned char)"




class StructInstance:

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)




class Struct(AbstractItem):

    DESCRIPTION = "Composition of other items (struct)"

    def __init__(self, instance_type, description, *args):
        default_value = instance_type()
        self.C_TYPE = ""
        for name, item in args:
            setattr(default_value, name, item.default_value)
            self.C_TYPE += item.C_TYPE
        AbstractItem.__init__(self, default_value, description)
        self.instance_ctor = instance_type
        self.content = args


    def serialize(self, value, buf):
        for name, item in self.content:
            item.serialize(getattr(value, name), buf)


    def deserialize(self, iterator):
        instance = self.instance_ctor()
        return self.deserialize_to(instance, iterator)


    def deserialize_to(self, instance, iterator):
        for name, item in self.content:
            value = item.deserialize(iterator)
            setattr(instance, name, value)
        return instance


    def serialize_as_text(self, instance):
        text = ""
        for name, item in self.content:
            text += " " + item.to_dump_as_text(getattr(instance, name))
        return text


    def to_dump(self, value):
        return '({})'.format(
            ', '.join("('{name}', {val})".format(
                name=name,
                val=item.to_dump(getattr(value,name))) for name,item in self.content
            )
        )


    def from_dump(self, value):
        instance = instance_ctor()
        for name, item in self.content:
            for vname, vvalue in value:
                if vname == name:
                    setattr(instance, name, item.from_dump(vvalue))
                    break
        return instance




class List(AbstractItem):

    DESCRIPTION = "List of max_count other items"

    def __init__(self, max_count, element_type, default_value = [], description = None):
        AbstractItem.__init__(self, default_value, description)
        self.max_count = max_count
        self.element_type = element_type
        self.C_TYPE = "B"
        for i in range(self.max_count):
            self.C_TYPE += self.element_type.C_TYPE


    def serialize(self, value, buf):
        count = min(len(value), self.max_count)
        buf.append(count)
        it = iter(value)
        for i in range(count):
            self.element_type.serialize(next(it), buf)
        # Serialize empty items
        for k in range(self.max_count - count):
            self.element_type.serialize(self.element_type.default_value, buf)


    def deserialize(self, iterator):
        l = []
        count = next(iterator)
        for i in range(count):
            l.append(self.element_type.deserialize(iterator))
        # Deserialize remaining items
        for i in range(self.max_count - count):
            self.element_type.deserialize(iterator)
        return l


    def to_dump(self, value):
        l = "["
        first = True
        for v in value:
            if first:
                first = False
            else:
                l += ", "
            l += self.element_type.to_dump(v)
        return l + "]"


    def from_dump(self, value):
        l = []
        count = min(len(value), self.max_count)
        it = iter(value)
        for i in range(count):
            l.append(self.element_type.from_dump(next(it)))
        return l




class String(AbstractItem):

    DESCRIPTION = "String of specified length"

    def __init__(self, length, default_value = "", description = None):
        AbstractItem.__init__(self, default_value, description)
        self.length = length
        self.C_TYPE = "{}s".format(length)
        self.DESCRIPTION = "String of length {}".format(self.length)


    def serialize(self, value, buf):
        buf.append(bytes(value,'utf-8','ignore'))


    def deserialize(self, iterator):
        val = super().deserialize(iterator)
        val = val.decode('utf-8')
        ind = val.find('\0')
        if ind != -1 :
            val = val[:ind]
        return val


    def to_dump(self, value):
        return "'" + value + "'"
