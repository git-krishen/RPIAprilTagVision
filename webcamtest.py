# Credit: Lobrien
import cv2
import robotpy_apriltag
import numpy
from wpimath.geometry import Pose3d
from cscore import CameraServer
from ntcore import NetworkTableInstance
import multiprocessing
from time import sleep

team = 5052
server = True
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

def initNetworkTable():
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
    foundTagsEntry = table.getEntry("foundTags")
    idQueryEntry.setInteger(idQueryEntry.getInteger(0))
    
    return tagIdEntry, translationEntry, rotationEntry, centerEntry, idQueryEntry, foundEntry, foundTagsEntry

def initCaptureAndEstimator():
    cvSink, outputStream = startCamera(0)
    image = numpy.zeros((xRes, yRes, 3), dtype = "uint8")
    detector, estimator = get_apriltag_detector_and_estimator((xRes, yRes))
    return cvSink, detector, estimator, image, outputStream

# Display loop: captures a frame, detects apriltags, puts frame in outputstream, and publishes pose data to networktables
def show_capture():
    # start NetworkTables
    tagIdEntry, translationEntry, rotationEntry, centerEntry, idQueryEntry, foundEntry, foundTagsEntry = initNetworkTable()
    cvSink, detector, estimator, image, outputStream = initCaptureAndEstimator()
    while True:
        print("executed")
        # Capture frame-by-frame
        time, frame = cvSink.grabFrame(image)
        # Detect apriltag
        frame_with_maybe_apriltags, results = detect_and_process_apriltag(frame, detector, estimator)

        outputStream.putFrame(frame_with_maybe_apriltags)

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
        foundTags = []
        for result in results :
            foundTags.append(result)
        foundTagsEntry.setIntegerArray(foundTags)
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


def startCamera(cameraPort):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format("Webcam", "/dev/video0"))
    # camera = UsbCamera("Webcam", "/dev/video0")
    camera = CameraServer.startAutomaticCapture(cameraPort)
    camera.setResolution(xRes, yRes)
    # camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    cvSink = CameraServer.getVideo()
    cvSink.setSource(camera)
    outputStream = CameraServer.putVideo("WebcamVid", xRes, yRes)
    return cvSink, outputStream

def runRingDetector():
    cvSink, outputStream = startCamera(1)
    image = numpy.zeros((xRes, yRes, 3), dtype = "uint8")
    ORANGE_MIN = numpy.array([1, 95, 150])
    ORANGE_MAX = numpy.array([27, 255, 255])
    while True:
        print("second func")
        time, frame = cvSink .grabFrame(image)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frame, ORANGE_MIN, ORANGE_MAX)
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        big_ellipse = None
        max_area = 0
        for contour in contours:
            if len(contour) >= 5:
                ellipse = cv2.fitEllipse(contour)
                width = ellipse[1][0]
                height = ellipse[1][1]
                area = width * height * numpy.pi # Calculate the area of the fitted ellipse
                if area > max_area and width > 10 and height > 10:
                    max_area = area
                    big_ellipse = ellipse


        
        cv2.ellipse(frame, big_ellipse, (0,255,0), 2,2)
        cv2.imshow('final', frame)
        cv2.waitKey(1)

        #TODO figure out OpenCV camera calibration
        #TODO implement matrix to find angle to camera of object



# Main function:
# Initializes capture & display window, initializes Apriltag detection, shows capture, cleans up
def main():
    
    first = multiprocessing.Process(target=show_capture)
    second = multiprocessing.Process(target=runRingDetector)
    # first.start()
    second.start()
    # first.join()
    second.join()

if __name__ == '__main__':
    main()
