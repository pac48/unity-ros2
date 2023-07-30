import os
import queue
import argparse
from jinja2 import Template
import re


class JinjaTemplate:
    templates = dict()

    @staticmethod
    def load(lang):
        JinjaTemplate.templates = get_all_templates(lang)


class Msg:
    def __init__(self, msg_type, ros_type):
        self.type = msg_type
        self.ros_type = ros_type
        self.fields = []


class Field:
    def __init__(self, msg_type, name, length, default_value, known_conversions):
        self.msg_type = msg_type
        self.name = name
        self.length = length
        self.default_value = default_value
        self.known_conversions = known_conversions
        if self.msg_type in self.known_conversions:
            self.declaration = self.known_conversions[self.msg_type](self.name, self.length)
        else:
            self.declaration = convert_custom_cpp(self.msg_type, self.name, self.length)

    # def __str__(self):
    #     if self.msg_type in self.known_conversions:
    #         return self.known_conversions[self.msg_type](self.name, self.length)
    #     else:
    #         return convert_custom_cpp(self.msg_type, self.name, self.length)


class StructCPP:
    def __init__(self, name, fields, msg_type, ros_msg_type, all_custom_types):
        self.name = name
        self.fields = fields
        self.msg_type = msg_type
        self.ros_msg_type = ros_msg_type
        self.all_custom_types = all_custom_types

    def __str__(self):
        j2_template = Template(JinjaTemplate.templates["struct"])
        data = {'name': self.name, 'fields': self.fields, 'msg_type': self.msg_type, 'ros_msg_type': self.ros_msg_type, 'all_custom_types': self.all_custom_types}
        return j2_template.render(data, trim_blocks=True)


def convert_int8_cpp(string, length):
    return f"int8_t"


def convert_uint8_cpp(string, length):
    return f"uint8_t"


def convert_int16_cpp(string, length):
    return f"int16_t"


def convert_uint16_cpp(string, length):
    return f"uint16_t"


def convert_int32_cpp(string, length):
    return f"int32_t"


def convert_uint32_cpp(string, length):
    return f"uint32_t"


def convert_int64_cpp(string, length):
    return f"long"


def convert_uint64_cpp(string, length):
    return f"unsigned long"


def convert_string_cpp(string, length):
    return f"char *"


def convert_float32_cpp(string, length):
    return f"float"


def convert_float64_cpp(string, length):
    return f"double"


def convert_bool_cpp(string, length):
    return f"bool"


def convert_char_cpp(string, length):
    return f"char"


def convert_byte_cpp(string, length):
    return f"uint8_t"


def convert_custom_cpp(custom_type, name, length):
    return custom_type


def create_struct_cpp(msg, known_conversions, all_custom_types):
    # name = msg.type.replace('/', '_')
    # fields = []
    # for field in msg.fields:
    #     fields.append(Field(field[0], field[1], field[2], known_conversions))

    return StructCPP(msg.type, msg.fields, msg.type, msg.ros_type, all_custom_types)


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


def get_all_templates(language):
    template_lang_path = os.path.join(
        os.path.dirname(__file__), "jinja_templates", language
    )
    if language == "cpp":
        template_markdown_path = os.path.join(
            os.path.dirname(__file__), "jinja_templates", "cpp"
        )
        template_paths = [template_lang_path, template_markdown_path]
    elif language == "c#":
        template_rst_path = os.path.join(
            os.path.dirname(__file__), "jinja_templates", "c#"
        )
        template_paths = [template_lang_path, template_rst_path]
    else:
        raise ValueError("language must be 'cpp' or 'c#'")

    template_map = {}
    for template_path in template_paths:
        for file_name in [
            f
            for f in os.listdir(template_path)
            if os.path.isfile(os.path.join(template_path, f))
        ]:
            with open(os.path.join(template_path, file_name)) as file:
                template_map[file_name] = file.read()

    return template_map


def convert_messages(create_struct, known_conversions):
    all_custom_types = set()
    msgs_map = dict()
    includes = []

    message_packages = ['std_msgs', 'geometry_msgs', 'sensor_msgs', 'nav_msgs', 'builtin_interfaces']
    for package in message_packages:
        directory_path = f'common_interfaces/{package}/msg/'
        file_list = os.listdir(directory_path)
        for file in file_list:
            msg_file = os.path.join(directory_path, file)
            with open(msg_file, 'r') as f:
                msg_type = f'{package}_{file.replace(".msg", "")}'
                ros_msg_type = f'{package}::msg::{file.replace(".msg", "")}'
                includes.append(ros_msg_type)
                msgs_map[msg_type] = Msg(msg_type, ros_msg_type)

                if msg_type not in known_conversions:
                    all_custom_types.add(msg_type)
                for line in f:
                    line = line.strip()
                    if len(line) == 0: continue
                    if line[0] == '#': continue
                    if line.__contains__('='):
                        line2 = line.split('=')
                        tmp = [v for v in line2[0].split()[:2]]
                        # tmp[1] += ' = ' + line2[1].split()[0]
                        default_value = line2[1].split()[0]
                    else:
                        tmp = [v for v in line.split()[:2]]
                        default_value = ''

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
                        all_custom_types.add(tmp[0])

                    # default_value
                    field = Field(tmp[0], tmp[1], tmp[2], default_value, known_conversions)
                    msgs_map[msg_type].fields.append(field)

    converted_msgs = set()
    structs = []
    q = queue.Queue()
    for t in all_custom_types:
        q.put(t)

    while not q.empty():
        key = q.get()

        if key in msgs_map:
            msg = msgs_map[key]
            none_missing = True
            for field in msg.fields:
                field_type = field.msg_type
                none_missing = none_missing and (field_type in known_conversions or field_type in converted_msgs)
            if not none_missing:
                q.put(key)
                continue

            converted_msgs.add(msg.type)
            out = create_struct(msg, known_conversions, all_custom_types)
            structs.append(out)

    def camel_to_snake(camel_str):
        # Replace capital letters with underscores followed by lowercase letters
        if camel_str.__contains__("DOF"):
            camel_str = camel_str.replace("DOF", "DOF_")
        if camel_str.__contains__("UI"):
            camel_str = camel_str.replace("UI", "U_I")
        snake_str = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', camel_str)
        # Convert the string to lowercase
        snake_str = snake_str.lower()
        return snake_str

    includes = [camel_to_snake(v.replace('::', '/')) for v in includes]

    j2_template = Template(JinjaTemplate.templates["struct_header"])
    data = {'structs': structs, 'includes': includes}
    return j2_template.render(data, trim_blocks=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="A Python argument parser that takes in two arguments.")

    # Add arguments
    parser.add_argument("output", type=str, help="location to save output file")
    parser.add_argument("language", type=str, help="generate code language: c++ or c#")

    args = parser.parse_args()
    if args.language not in {'cpp', 'c#'}:
        raise ValueError("Argument 2 must be equal to 'cpp' or 'c#'")
    if args.language == 'cpp':
        JinjaTemplate.load('cpp')
        contents = convert_messages(create_struct_cpp, known_conversions_cpp)
    else:
        raise NotImplementedError()
        # contents = convert_messages(create_struct_cpp, known_conversions_cpp)
    # print(contents)
    with open(args.output, 'w') as f:
        f.write(contents)
