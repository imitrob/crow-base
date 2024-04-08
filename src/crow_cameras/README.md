# Camera Node

## Inputs

- Hardware IntelliSense Camera(s) OR None if in fake mode

## Outputs

- For each camera:
	- Topic `/cameraX/color/image_raw`
	- Topic `/cameraX/color/camera_info`
	- Topic `/cameraX/depth/image_raw`
	- Topic `/cameraX/depth/camera_info`

## CLI flags

- `force_fake:=True` - no matter what, use fake mode
- `force_real:=True` - use real camera, crash if none is found

### Examples

```bash
ros2 launch crow_cameras cameras.launch.py "force_fake:=True"
ros2 launch crow_cameras cameras.launch.py "force_real:=True" 
```

## Used Hardware 

*( as discovered by lsusb )*

- Intel Corp. RealSense D435