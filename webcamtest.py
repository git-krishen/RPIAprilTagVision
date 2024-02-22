# Credit: Lobrien
import cv2
import robotpy_apriltag
from wpimath.geometry import Pose3d
import numpy
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode
from ntcore import NetworkTableInstance, GenericPublisher, GenericEntry, EventFlags, Topic, PubSubOptions

team = 5052
server = True
xRes = 640
yRes = 480
DETECTION_MARGIN_THRESHOLD = 100

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
    # hamming = tag.getHamming()
    # decision_margin = tag.getDecisionMargin()
    # print("Hamming for {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))

    est = estimator.estimateOrthogonalIteration(tag, 50)
    return tag_id, est.pose1, center

# This function is called once for every frame captured by the Webcam. For testing, it can simply
# be passed a frame capture loaded from a file. (See commented-out alternative `if __name__ == main:` at bottom of file)
def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    # Note that results will be empty if no apriltag is detected
    # for result in results:
    #     # frame = draw_tag(frame, result)
    #     print(result)

    return frame, results

# Display loop: captures a frame, detects apriltags, puts frame in outputstream, and publishes pose data to networktables
def show_capture(cvSink, detector, estimator, image, outputstream, tagEntry, table):
    while True:
        # Capture frame-by-frame
        time, frame = cvSink.grabFrame(image)
        # Detect apriltag
        frame_with_maybe_apriltags, results = detect_and_process_apriltag(frame, detector, estimator)

        outputstream.putFrame(frame_with_maybe_apriltags)

        for result in results :
            tagId = result[0]
            translation = result[1].translation()
            rotation = result[1].rotation()
            center = result[2]

            # wEntry.set(tagId)
            # xEntry.set([translation.X(), translation.Y(), translation.Z()])
            # yEntry.set([rotation.X(), rotation.Y(), rotation.Z()])
            # zEntry.set([center.x, center.y])
            table.putNumberArray("apriltag", [tagId, translation.X(), 
                                              translation.Y(), translation.Z(), 
                                              rotation.X(), rotation.Y(), rotation.Z(), 
                                              center.x, center.y])

            # table.putNumber("tagID", tagId)
            # table.putNumberArray("translation", [translation.X(), translation.Y(), translation.Z()])
            # table.putNumberArray("rotation", [rotation.X(), rotation.Y(), rotation.Z()])
            # table.putNumberArray("center", [center.x, center.y])
            aprilTag = tagEntry.getDoubleArray([])
            print(f"Tag ID: {aprilTag[0]},  Translation (X, Y, Z): [{aprilTag[1]}, {aprilTag[2]}, {aprilTag[3]}] Rotation (X, Y, Z): [{aprilTag[4]}, {aprilTag[5]}, {aprilTag[6]}] Center Point: [{aprilTag[7]}, {aprilTag[8]}] @ {time}")

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
    return cvSink, outputStream

# Main function:
# Initializes capture & display window, initializes Apriltag detection, shows capture, cleans up
def main():
    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
        table = ntinst.getTable("datatable")
        tagEntry = table.getEntry("apriltag")
        # wEntry = table.getDoubleTopic("tagID").publish()
        # xEntry = table.getDoubleArrayTopic("translation").publish()
        # yEntry = table.getDoubleArrayTopic("rotation").publish()
        # zEntry = table.getDoubleArrayTopic("center").publish()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()
        table = ntinst.getTable("datatable")
        tagEntry = table.getEntry("apriltag")
    cvSink, outputStream = startCamera()
    image = numpy.zeros((xRes, yRes, 3), dtype = "uint8")
    detector, estimator = get_apriltag_detector_and_estimator((xRes, yRes))
    show_capture(cvSink, detector, estimator, image, outputStream, tagEntry, table)

if __name__ == '__main__':
    main()
