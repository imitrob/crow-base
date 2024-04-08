import yaml
import crow_utils.crow_config as crow_config


class CameraGlobalTFGetter:

    DEFAULT_CONFIG_FILE = "crow_cameras/camera_transformation_data.yaml"
    UNKNOWN_CAMERA_TRANSFORM = "0 0 0 0 0 0 1"

    def __init__(self, config_file=None):

        if config_file is None:
            config_file = self.DEFAULT_CONFIG_FILE

        self.config_file = config_file
        self.data = crow_config.load_yaml_config(config_file)

    def get_camera_transformation(self, serial_nr, cast=False):

        transform=None

        if serial_nr[0] == "_":
            serial_nr = serial_nr[1:]
        if serial_nr in self.data:
            transform = self.data[serial_nr]
        else:
            transform = CameraGlobalTFGetter.UNKNOWN_CAMERA_TRANSFORM
        
        if cast:
            transform = transform.split(" ")
            transform = [float(x) for x in transform]
        
        return transform
