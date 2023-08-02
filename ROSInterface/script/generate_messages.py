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
        self.lp_index = -1
        if self.msg_type in self.known_conversions:
            self.declaration = self.known_conversions[self.msg_type]
        else:
            self.declaration = convert_custom_cpp(self.msg_type, self.name, self.length)


class StructCPP:
    def __init__(self, name, fields, msg_type, ros_msg_type, all_custom_types):
        self.name = name
        self.fields = fields
        counter = len(fields) - 1
        for f in fields:
            if f.length == -1:
                counter += 1
                f.lp_index = counter

        self.msg_type = msg_type
        self.ros_msg_type = ros_msg_type
        self.all_custom_types = all_custom_types

    def __str__(self):
        j2_template = Template(JinjaTemplate.templates["struct"])
        data = {'name': self.name, 'fields': self.fields, 'msg_type': self.msg_type, 'ros_msg_type': self.ros_msg_type,
                'all_custom_types': self.all_custom_types}
        return j2_template.render(data, trim_blocks=True)


def convert_custom_cpp(custom_type, name, length):
    return custom_type


def create_struct_cpp(msg, all_custom_types):
    return StructCPP(msg.type, msg.fields, msg.type, msg.ros_type, all_custom_types)


def create_struct_cs(msg, all_custom_types):
    return StructCPP(msg.type, msg.fields, msg.type, msg.ros_type, all_custom_types)


known_conversions_cpp = {
    "int8": f"int8_t",
    "uint8": f"uint8_t",
    "int16": f"int16_t",
    "uint16": f"uint16_t",
    "int32": f"int32_t",
    "uint32": f"uint32_t",
    "int64": f"long",
    "uint64": f"unsigned long",
    "float32": f"float",
    "float64": f"double",
    "string": f"char *",
    "bool": f"bool",
    "char": f"char",
    "byte": f"uint8_t",
}

known_conversions_cs = {
    "int8": f"sbyte",
    "uint8": f"byte",
    "int16": f"short",
    "uint16": f"ushort",
    "int32": f"int",
    "uint32": f"uint",
    "int64": f"long",
    "uint64": f"ulong",
    "float32": f"float",
    "float64": f"double",
    "string": f"IntPtr",
    "bool": f"bool",
    "char": f"char",
    "byte": f"byte",
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
            out = create_struct(msg, all_custom_types)
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

    class TypePair:
        def __init__(self, struct_type, ros_type):
            self.struct_type = struct_type
            self.ros_type = ros_type

    all_ros_message_types = [TypePair(key, msgs_map[key].ros_type) for key in msgs_map]

    includes = [camel_to_snake(v.replace('::', '/')) for v in includes]

    j2_template = Template(JinjaTemplate.templates["struct_header"])
    data = {'structs': structs, 'includes': includes, 'all_ros_message_types': all_ros_message_types}
    return j2_template.render(data, trim_blocks=True), all_ros_message_types


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="A Python argument parser that takes in two arguments.")

    # Add arguments
    parser.add_argument("language", type=str, help="location to save output file")
    parser.add_argument("--header_dir", type=str, help="generate code language: c++ or c#")
    parser.add_argument("--src_dir", type=str, help="generate code language: c++ or c#")
    parser.add_argument("--csharp_dir", type=str, help="generate code language: c++ or c#")

    args = parser.parse_args()
    if args.language == 'cpp':
        JinjaTemplate.load('cpp')
        contents, all_ros_message_types = convert_messages(create_struct_cpp, known_conversions_cpp)
        if not os.path.exists(os.path.join(args.header_dir)):
            os.makedirs(os.path.join(args.header_dir))
        if not os.path.exists(os.path.join(args.src_dir)):
            os.makedirs(os.path.join(args.src_dir))
        with open(os.path.join(args.header_dir, 'tmp.h'), 'w') as f:
            f.write(contents)
        for msg in all_ros_message_types:
            j2_template = Template(JinjaTemplate.templates["sub_pub_header"])
            data = {'msg': msg}
            out = j2_template.render(data, trim_blocks=True)
            with open(os.path.join(args.header_dir, msg.struct_type + '.h'), 'w') as f:
                f.write(out)
            j2_template = Template(JinjaTemplate.templates["sub_pub_impl"])
            out = j2_template.render(data, trim_blocks=True)
            with open(os.path.join(args.src_dir, msg.struct_type + '.cpp'), 'w') as f:
                f.write(out)

        j2_template = Template(JinjaTemplate.templates["sub_pub_interface_impl"])
        includes = [f'generated/{v.struct_type}' for v in all_ros_message_types]
        data = {'all_ros_message_types': all_ros_message_types, 'includes': includes}
        out = j2_template.render(data, trim_blocks=True)
        with open(os.path.join(args.src_dir, 'sub_pub_interface.cpp'), 'w') as f:
            f.write(out)

    elif args.language == 'c#':
        JinjaTemplate.load('c#')
        contents, all_ros_message_types = convert_messages(create_struct_cs, known_conversions_cs)
        if not os.path.exists(os.path.join(args.csharp_dir)):
            os.makedirs(os.path.join(args.csharp_dir))
        with open(os.path.join(args.csharp_dir, 'message_structs.cs'), 'w') as f:
            f.write(contents)

    else:
        raise ValueError("Argument 2 must be equal to 'cpp' or 'c#'")

    # contents = convert_messages(create_struct_cpp, known_conversions_cpp)
    # print(contents)
