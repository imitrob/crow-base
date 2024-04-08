import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_record_to_file("object_detection.bag")

# Start streaming
pipeline.start(config)

e1 = cv2.getTickCount()


# Setup video recoding
# Define the codec and create VideoWriter object
out = cv2.VideoWriter(
    "outpy.avi", cv2.VideoWriter_fourcc("M", "J", "P", "G"), 10, (640, 480),
)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())

        #color_frame_resized = cv2.resize(color_image, (640, 480))
        #out.write(color_frame_resized)

        # Apply colormap on depth image (image must be converted to
        # 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(
        #     cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        # )

        # Stack both images horizontally
        # images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("RealSense", images)

        print(color_image)
        print(type(color_image))
        print(color_image.shape)
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("RealSense", color_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        e2 = cv2.getTickCount()
        t = (e2 - e1) / cv2.getTickFrequency()
        if t > 30:  # change it to record what length of video you are interested in
            print("Done!")
            break

finally:

    # Stop streaming
    pipeline.stop()

out.release()
cv2.destroyAllWindows()
