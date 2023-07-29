import os
import queue


def convert_int8(string, length):
    if length == 1:
        return f"char {string};\n"
    elif length == -1:
        return f"char {string}[];\n"
    else:
        return f"char {string}[{length}];\n"


def convert_uint8(string, length):
    if length == 1:
        return f"unsigned char {string};\n"
    elif length == -1:
        return f"unsigned char {string}[];\n"
    else:
        return f"unsigned char {string}[{length}];\n"


def convert_int16(string, length):
    if length == 1:
        return f"short {string};\n"
    elif length == -1:
        return f"short {string}[];\n"
    else:
        return f"short {string}[{length}];\n"


def convert_uint16(string, length):
    if length == 1:
        return f"unsigned short {string};\n"
    elif length == -1:
        return f"unsigned short {string}[];\n"
    else:
        return f"unsigned short {string}[{length}];\n"


def convert_int32(string, length):
    if length == 1:
        return f"int {string};\n"
    elif length == -1:
        return f"int {string}[];\n"
    else:
        return f"int {string}[{length}];\n"


def convert_uint32(string, length):
    if length == 1:
        return f"unsigned int {string};\n"
    elif length == -1:
        return f"unsigned int {string}[];\n"
    else:
        return f"unsigned int {string}[{length}];\n"


def convert_int64(string, length):
    if length == 1:
        return f"long {string};\n"
    elif length == -1:
        return f"long {string}[];\n"
    else:
        return f"long {string}[{length}];\n"


def convert_uint64(string, length):
    if length == 1:
        return f"unsigned long {string};\n"
    elif length == -1:
        return f"unsigned long {string}[];\n"
    else:
        return f"unsigned long {string}[{length}];\n"


def convert_string(string, length):
    if length == 1:
        return f"char * {string};\n"
    elif length == -1:
        return f"char * {string}[];\n"
    else:
        return f"char * {string}[{length}];\n"


def convert_float32(string, length):
    if length == 1:
        return f"float {string};\n"
    elif length == -1:
        return f"float {string}[];\n"
    else:
        return f"float {string}[{length}];\n"


def convert_float64(string, length):
    if length == 1:
        return f"double {string};\n"
    elif length == -1:
        return f"double {string}[];\n"
    else:
        return f"double {string}[{length}];\n"


def convert_bool(string, length):
    if length == 1:
        return f"bool {string};\n"
    elif length == -1:
        return f"bool {string}[];\n"
    else:
        return f"bool {string}[{length}];\n"


def convert_char(string, length):
    if length == 1:
        return f"char {string};\n"
    elif length == -1:
        return f"char {string}[];\n"
    else:
        return f"char {string}[{length}];\n"


def convert_byte(string, length):
    if length == 1:
        return f"char {string};\n"
    elif length == -1:
        return f"char {string}[];\n"
    else:
        return f"char {string}[{length}];\n"


def convert_custom(custom_type, string, length):
    if length == 1:
        return custom_type + ' ' + string + ';\n'
    elif length == -1:
        return custom_type + ' ' + string + f'{[]};\n'
    else:
        return custom_type + ' ' + string + f'{[length]};\n'


def create_struct(msg):
    out = 'struct ' + msg.type.replace('/', '_') + ' {\n'
    for field in msg.fields:
        if field[0] in known_conversions:
            out += known_conversions[field[0]](field[1], field[2])
        else:
            out += convert_custom(field[1], field[1], field[2])
    out += '};'
    return out


class Msg:
    def __init__(self, msg_type):
        self.type = msg_type
        self.fields = []


known_conversions = {
    "int8": convert_int8,
    "uint8": convert_uint8,
    "int16": convert_int16,
    "uint16": convert_uint16,
    "int32": convert_int32,
    "uint32": convert_uint32,
    "int64": convert_int64,
    "uint64": convert_uint64,
    "float32": convert_float32,
    "float64": convert_float64,
    "string": convert_string,
    "bool": convert_bool,
    "char": convert_char,
    "byte": convert_byte,
}

create_struct_func = create_struct


def convert_messages():
    types = set()
    msgs_map = dict()

    message_packages = ['std_msgs', 'geometry_msgs', 'sensor_msgs', 'nav_msgs', 'builtin_interfaces']
    for package in message_packages:
        directory_path = f'common_interfaces/{package}/msg/'
        file_list = os.listdir(directory_path)
        for file in file_list:
            msg_file = os.path.join(directory_path, file)
            with open(msg_file, 'r') as f:
                msg_type = f'{package}_{file.replace(".msg", "")}'
                msgs_map[msg_type] = Msg(msg_type)
                if msg_type not in known_conversions:
                    types.add(msg_type)
                for line in f:
                    line = line.strip()
                    if len(line) == 0: continue
                    if line[0] == '#': continue
                    tmp = [v for v in line.split()[:2]]
                    if tmp[0].__contains__('['):
                        val = tmp[0].split('[')[1].split(']')[0]
                        if len(val) > 0:
                            num = int(val)
                        else:
                            num = -1
                        tmp.append(num)
                        tmp[0] = tmp[0].split('[')[0]
                    else:
                        tmp.append(1)

                    # print(tmp)
                    if tmp[0] not in known_conversions:
                        if not tmp[0].__contains__('/'):
                            tmp[0] = package + '_' + tmp[0].split('/')[-1]
                        else:
                            tmp[0] = tmp[0].replace('/', '_')
                        types.add(tmp[0])

                    msgs_map[msg_type].fields.append(tmp)

    converted_msgs = dict()
    q = queue.Queue()
    for t in types:
        q.put(t)

    while not q.empty():
        key = q.get()

        if key in msgs_map:
            msg = msgs_map[key]
            none_missing = True
            for field in msg.fields:
                field_type = field[0].replace('/', '_')
                none_missing = none_missing and (field_type in known_conversions or field_type in converted_msgs)
            if not none_missing:
                q.put(key)
                continue

            out = create_struct_func(msg)
            print(out)
            converted_msgs[msg.type] = out


convert_messages()
