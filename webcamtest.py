# Credit: Lobrien
import cv2
import robotpy_apriltag
# import robotpy
from wpimath.geometry import Pose3d
# import math

# import json
# import time
# import sys

import numpy
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode
from ntcore import NetworkTableInstance, GenericPublisher, GenericEntry, EventFlags, Topic, PubSubOptions

team = 5052
server = True
xRes = 640
yRes = 480

# This function is called once to initialize the apriltag detector and the pose estimator
def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    assert detector.addFamily("tag36h11")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(robotpy_apriltag.AprilTagPoseEstimator.Config(0.2, 500, 500, frame_size[1] / 2.0, frame_size[0] / 2.0))
    return detector, estimator
    
# This function is called for every detected tag. It uses the `estimator` to 
# return information about the tag, including its centerpoint. (The corners are 
# also available.)
def process_apriltag(estimator, tag):
    tag_id = tag.getId()
    center = tag.getCenter()
    hamming = tag.getHamming()
    decision_margin = tag.getDecisionMargin()
    # print("Hamming for {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))

    est = estimator.estimateOrthogonalIteration(tag, 50)
    return tag_id, est.pose1, center

# This simply outputs some information about the results returned by `process_apriltag`.
# It prints some info to the console and draws a circle around the detected center of the tag
def draw_tag(frame, result):
    assert frame is not None
    assert result is not None
    tag_id, pose, center = result
    print(center)
    cv2.circle(frame, (int(center.x), int(center.y)), 50, (255, 0, 255), 3)
    msg = f"Tag ID: {tag_id} Pose: {pose}"
    cv2.putText(frame, msg, (100, 50 * 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

# This function is called once for every frame captured by the Webcam. For testing, it can simply
# be passed a frame capture loaded from a file. (See commented-out alternative `if __name__ == main:` at bottom of file)
def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    DETECTION_MARGIN_THRESHOLD = 100
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    # Note that results will be empty if no apriltag is detected
    for result in results:
        frame = draw_tag(frame, result)
        print(result)

    return frame, results


# Draws a "targeting overlay" on `frame`
def draw_overlay(frame):
    # Get the height and width of the frame
    height, width, channels = frame.shape
    # Draw a circle in the center of the frame
    cv2.circle(frame, (width // 2, height // 2), 50, (0, 0, 255), 1)
    # Draw diagonal lines from top-left to bottom-right and top-right to bottom-left
    cv2.line(frame, (0, 0), (width, height), (0, 255, 0), 1)
    cv2.line(frame, (width, 0), (0, height), (0, 255, 0), 1)
    # Draw a text on the frame
    cv2.putText(frame, 'q to quit', (width//2 - 100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return frame


# # Called once at program end
# def cleanup_capture(capture):
#     # When everything done, release the capture
#     capture.release()
#     cv2.destroyAllWindows()

# Display loop: captures a frame, detects apriltags, draws an overlay, shows the composited frame
# Infinite loop breaks when user presses 'q' key on keyboard
def show_capture(capture_window_name, capture, detector, estimator, image, outputstream, ntinst, wEntry, xEntry, yEntry, zEntry, table):
    while True:
        # Capture frame-by-frame
        time, frame = capture.grabFrame(image)
        # frame = cv2.Mat(frame)
        # Detect apriltag
        frame_with_maybe_apriltags, results = detect_and_process_apriltag(frame, detector, estimator)

        overlaid_image = draw_overlay(frame_with_maybe_apriltags)
        # Display the resulting frame in the named window
        # cv2.imshow(capture_window_name, overlaid_image)
        outputstream.putFrame(overlaid_image)
        
        for result in results :
            tagID = result[0]
            translation = result[1].translation()
            rotation = result[1].rotation()
            center = result[2]
            table.putNumber("tagID", tagID)
            table.putNumberArray("translation", [translation.X(), translation.Y(), translation.Z()])
            table.putNumberArray("rotation", [rotation.X(), rotation.Y(), rotation.Z()])
            table.putNumberArray("center", [center.x, center.y])
            # w = wEntry.getDouble(0.0)
            # x = xEntry.getDoubleArray(0.0)
            # y = yEntry.getDoubleArray(0.0)
            # z = zEntry.getDoubleArray(0.0)
            # print("Tag ID: ", w, " Translation (X, Y, Z): ", x, "Rotation (X, Y, Z)", y, " Center Point: ", z)

        

        #<ntcore._ntcore.Topic object at 0x000002606A35C1B0>
        # print(ntinst.getTable("datatable"))
        # print(ntinst.getEntry("default"))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def startCamera():
    """Start running the camera."""
    print("Starting camera '{}' on {}".format("Webcam", "/dev/video0"))
    # camera = UsbCamera("Webcam", "/dev/video0")
    camera = CameraServer.startAutomaticCapture(2)
    camera.setResolution(xRes, yRes)
    # camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    cvSink = CameraServer.getVideo()
    cvSink.setSource(camera)
    outputStream = CameraServer.putVideo("WebcamVid", xRes, yRes)
    return camera, cvSink, outputStream

# Main function:
# Initializes capture & display window, initializes Apriltag detection, shows capture, cleans up
def main():
    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
        table = ntinst.getTable("datatable")
        wEntry = table.getEntry("tagID")
        xEntry = table.getEntry("translation")
        yEntry = table.getEntry("rotation")
        zEntry = table.getEntry("center")
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    camera, cvSink, outputStream = startCamera()
    image = numpy.zeros((xRes, yRes, 3), dtype = "uint8")
    # print(image.shape, image.dtype)
    capture_window_name = 'Capture Window'
    detector, estimator = get_apriltag_detector_and_estimator((xRes, yRes))
    show_capture(capture_window_name, cvSink, detector, estimator, image, outputStream, ntinst, wEntry, xEntry, yEntry, zEntry, table)
    # cleanup_capture(capture)
if __name__ == '__main__':
    main()
    # frame = cv2.imread('../frc_image.png')
    # assert frame is not None
    # detector, estimator = get_apriltag_detector_and_estimator((640,480))
    # out_frame = detect_and_process_apriltag(frame, detector, estimator)
    # cv2.imwrite('out.jpg', out_frame)