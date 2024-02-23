# Credit: Lobrien
import cv2
import robotpy_apriltag
from wpimath.geometry import Pose3d
import numpy
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode
from ntcore import NetworkTableInstance, GenericPublisher, GenericEntry, EventFlags, Topic, PubSubOptions

team = 5052
server = False
xRes = 320
yRes = 224
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
def show_capture(cvSink, detector, estimator, image, outputstream, tagIdEntry, translationEntry, rotationEntry, centerEntry, idQueryEntry, foundEntry, table):
    while True:
        # Capture frame-by-frame
        time, frame = cvSink.grabFrame(image)
        # Detect apriltag
        frame_with_maybe_apriltags, results = detect_and_process_apriltag(frame, detector, estimator)

        outputstream.putFrame(frame_with_maybe_apriltags)

        """
        Check for an entry called "idQuery"
        If it's 0, find the first apriltag in results and push that to networktables
        If it's not 0, then if we see the apriltag with the ID in idQuery, then return pose, otherwise return default
        Also return boolean "found"
        """
               
        idQuery = idQueryEntry.getInteger(0)
        foundEntry.setBoolean(False)
        tagIdEntry.setInteger(0)
        translationEntry.setDoubleArray([0, 0, 0]) 
        rotationEntry.setDoubleArray([0, 0, 0])
        centerEntry.setDoubleArray([0, 0])
        foundEntry.setBoolean(False)
        if(idQuery > 0) :
            for result in results :
                tagId = result[0]
                if(tagId == idQuery) :
                    translation = result[1].translation()
                    rotation = result[1].rotation()
                    center = result[2]
                    tagIdEntry.setInteger(tagId)
                    translationEntry.setDoubleArray([translation.X(), translation.Y(), translation.Z()]) 
                    rotationEntry.setDoubleArray([rotation.X(), rotation.Y(), rotation.Z()])
                    centerEntry.setDoubleArray([center.x, center.y])
                    foundEntry.setBoolean(True)
                    break
        elif(idQuery == 0):
            if (len(results) > 0):
                tagId = results[0][0]
                translation = results[0][1].translation()
                rotation = results[0][1].rotation()
                center = results[0][2]
                tagIdEntry.setInteger(tagId)
                translationEntry.setDoubleArray([translation.X(), translation.Y(), translation.Z()]) 
                rotationEntry.setDoubleArray([rotation.X(), rotation.Y(), rotation.Z()])
                centerEntry.setDoubleArray([center.x, center.y])
                foundEntry.setBoolean(True)


def startCamera():
    # Start running the camera.
    print("Starting camera '{}' on {}".format("Webcam", "/dev/video0"))
    # camera = UsbCamera("Webcam", "/dev/video0")
    camera = CameraServer.startAutomaticCapture()
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
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()
        
    table = ntinst.getTable("apriltag")
    tagIdEntry = table.getEntry("tagID")
    translationEntry = table.getEntry("translation")
    rotationEntry = table.getEntry("rotation")
    centerEntry = table.getEntry("center")
    idQueryEntry = table.getEntry("idQuery")
    foundEntry = table.getEntry("found")
    
    idQueryEntry.setInteger(idQueryEntry.getInteger(0))
    cvSink, outputStream = startCamera()
    image = numpy.zeros((xRes, yRes, 3), dtype = "uint8")
    detector, estimator = get_apriltag_detector_and_estimator((xRes, yRes))
    show_capture(cvSink, detector, estimator, image, outputStream, tagIdEntry, translationEntry, rotationEntry, centerEntry, idQueryEntry, foundEntry, table)

if __name__ == '__main__':
    main()
