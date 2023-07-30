import os
import queue
import argparse


def convert_int8_cpp(string, length):
    if length == 1:
        return f"char {string};\n"
    elif length == -1:
        return f"char * {string};\nint {string}_length;\n"
    else:
        return f"char {string}[{length}];\nint {string}_length;\n"


def convert_uint8_cpp(string, length):
    if length == 1:
        return f"unsigned char {string};\n"
    elif length == -1:
        return f"unsigned char * {string};\nint {string}_length;\n"
    else:
        return f"unsigned char {string}[{length}];\nint {string}_length;\n"


def convert_int16_cpp(string, length):
    if length == 1:
        return f"short {string};\n"
    elif length == -1:
        return f"short * {string};\nint {string}_length;\n"
    else:
        return f"short {string}[{length}];\nint {string}_length;\n"


def convert_uint16_cpp(string, length):
    if length == 1:
        return f"unsigned short {string};\n"
    elif length == -1:
        return f"unsigned short * {string};\nint {string}_length;\n"
    else:
        return f"unsigned short {string}[{length}];\nint {string}_length;\n"


def convert_int32_cpp(string, length):
    if length == 1:
        return f"int {string};\n"
    elif length == -1:
        return f"int * {string};\nint {string}_length;\n"
    else:
        return f"int {string}[{length}];\nint {string}_length;\n"


def convert_uint32_cpp(string, length):
    if length == 1:
        return f"unsigned int {string};\n"
    elif length == -1:
        return f"unsigned int * {string};\nint {string}_length;\n"
    else:
        return f"unsigned int {string}[{length}];\nint {string}_length;\n"


def convert_int64_cpp(string, length):
    if length == 1:
        return f"long {string};\n"
    elif length == -1:
        return f"long * {string};\nint {string}_length;\n"
    else:
        return f"long {string}[{length}];\nint {string}_length;\n"


def convert_uint64_cpp(string, length):
    if length == 1:
        return f"unsigned long {string};\n"
    elif length == -1:
        return f"unsigned long * {string};\nint {string}_length;\n"
    else:
        return f"unsigned long {string}[{length}];\nint {string}_length;\n"


def convert_string_cpp(string, length):
    if length == 1:
        return f"char * {string};\n"
    elif length == -1:
        return f"char ** {string};\nint {string}_length;\n"
    else:
        return f"char ** {string}[{length}];\nint {string}_length;\n"


def convert_float32_cpp(string, length):
    if length == 1:
        return f"float {string};\n"
    elif length == -1:
        return f"float * {string};\nint {string}_length;\n"
    else:
        return f"float {string}[{length}];\nint {string}_length;\n"


def convert_float64_cpp(string, length):
    if length == 1:
        return f"double {string};\n"
    elif length == -1:
        return f"double * {string};\nint {string}_length;\n"
    else:
        return f"double {string}[{length}];\nint {string}_length;\n"


def convert_bool_cpp(string, length):
    if length == 1:
        return f"bool {string};\n"
    elif length == -1:
        return f"bool * {string};\nint {string}_length;\n"
    else:
        return f"bool {string}[{length}];\nint {string}_length;\n"


def convert_char_cpp(string, length):
    if length == 1:
        return f"char {string};\n"
    elif length == -1:
        return f"char * {string};\nint {string}_length;\n"
    else:
        return f"char {string}[{length}];\nint {string}_length;\n"


def convert_byte_cpp(string, length):
    if length == 1:
        return f"char {string};\n"
    elif length == -1:
        return f"char * {string};\nint {string}_length;\n"
    else:
        return f"char {string}[{length}];\nint {string}_length;\n"


def convert_custom_cpp(custom_type, string, length):
    if length == 1:
        return custom_type + ' ' + string + ';\n'
    elif length == -1:
        return custom_type + ' * ' + string + f';\nint {string}_length;\n'
    else:
        return custom_type + ' ' + string + f'{[length]};\nint {string}_length;\n'


def create_struct_cpp(msg, known_conversions):
    out = 'struct ' + msg.type.replace('/', '_') + ' {\n'
    for field in msg.fields:
        if field[0] in known_conversions:
            out += known_conversions[field[0]](field[1], field[2])
        else:
            out += convert_custom_cpp(field[0], field[1], field[2])
    out += '};'
    return out


class Msg:
    def __init__(self, msg_type):
        self.type = msg_type
        self.fields = []


known_conversions_cpp = {
    "int8": convert_int8_cpp,
    "uint8": convert_uint8_cpp,
    "int16": convert_int16_cpp,
    "uint16": convert_uint16_cpp,
    "int32": convert_int32_cpp,
    "uint32": convert_uint32_cpp,
    "int64": convert_int64_cpp,
    "uint64": convert_uint64_cpp,
    "float32": convert_float32_cpp,
    "float64": convert_float64_cpp,
    "string": convert_string_cpp,
    "bool": convert_bool_cpp,
    "char": convert_char_cpp,
    "byte": convert_byte_cpp,
}


def convert_messages(create_struct, known_conversions):
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
                    if line.__contains__('='):
                        line2 = line.split('=')
                        tmp = [v for v in line2[0].split()[:2]]
                        tmp[1] += ' = ' + line2[1].split()[0]
                    else:
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

    converted_msgs = set()
    contents = ""
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

            out = create_struct(msg, known_conversions)
            converted_msgs.add(msg.type)
            contents += out

    return contents

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="A Python argument parser that takes in two arguments.")

    # Add arguments
    parser.add_argument("output", type=str, help="location to save output file")
    parser.add_argument("language", type=str, help="generate code language: c++ or c#")

    args = parser.parse_args()
    if args.language not in {'cpp', 'c#'}:
        raise ValueError("Argument 2 must be equal to 'cpp' or 'c#'")
    if args.language == 'cpp':
        contents = convert_messages(create_struct_cpp, known_conversions_cpp)
    else:
        raise NotImplementedError()
        # contents = convert_messages(create_struct_cpp, known_conversions_cpp)
    # print(contents)
    with open(args.output,'w') as f:
        f.write(contents)
