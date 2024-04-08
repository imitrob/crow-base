import os
import yaml
from dotenv import load_dotenv
from importlib.util import find_spec

# CONFIG
DEFAULT_CONFIG_DIR="~/.local/share/crow"
#DEFAULT_LIB_DIR="~/.local/share/crow"
DEFAULT_LIB_DIR="~/crow-base/lib"
#

def get_lib_location(library):
    load_dotenv()

    lib_dir = os.path.expanduser(DEFAULT_CONFIG_DIR)

    if "CROW_LIB_DIR" in os.environ:
        lib_dir = os.path.expanduser(os.environ["CROW_LIB_DIR"])

    if not os.path.isdir(lib_dir):
        os.makedirs(lib_dir)

    lib_path = os.path.join(lib_dir, library)
    return lib_path


def get_config_file(name):
    load_dotenv()

    config_dir = os.path.expanduser(DEFAULT_CONFIG_DIR)

    if "CROW_CONFIG_DIR" in os.environ:
        config_dir = os.path.abspath(os.environ["CROW_CONFIG_DIR"])
        config_dir = os.path.expanduser(config_dir)

    if not os.path.isdir(config_dir):
        os.makedirs(config_dir)

    config_path = os.path.join(config_dir, name)
    return config_path


def load_yaml_config(name):
    config_path = get_config_file(name)

    with open(config_path, "r") as f:
        config_dict = yaml.load(f, Loader=yaml.SafeLoader)

    return config_dict

def dump_yaml_config(name,data):
    config_path = get_config_file(name)

    with open(config_path, "r") as f:
        config_dict = yaml.dump(data, stream = f, default_flow_style=False)

    return config_dict
    
def dump_yaml_file(f,data):
    config_dict = yaml.dump(data, stream = f, default_flow_style=False)

    return config_dict


def get_lib(name):
    load_dotenv()

    lib_dir = os.path.expanduser(DEFAULT_LIB_DIR)

    if "CROW_LIB_DIR" in os.environ:
        lib_dir = os.path.expanduser(os.environ["CROW_LIB_DIR"])

    if not os.path.isdir(lib_dir):
        os.makedirs(lib_dir)
    
    lib_name_dir = os.path.join(lib_dir, name)

    return lib_name_dir

def get_python_package_loc(package):
    return find_spec(package).submodule_search_locations[0]

def get_package_data_loc(package):
    return os.path.join(get_python_package_loc(package), "..", "data")
