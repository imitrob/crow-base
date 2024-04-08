# CROW Detector

## Inputs

Camera topics:
- `/cameraX/color/image_raw` for each camera

## Outputs

Camera topics:
- `/cameraX/detections/image_annot`
- `/cameraX/detections/masks`
- `/cameraX/detections/bboxes`

## CLI

Running

```bash
ros2 run crow_detector detector
```