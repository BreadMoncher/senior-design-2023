import serial
import depthai as dai
import queue

teraranger_data = queue.Queue(maxsize=0)
telemetry_received_data = queue.Queue(maxsize=0)
telemetry_send_data = queue.Queue(maxsize=0)
depth_data = queue.Queue(maxsize=1)

def teraranger_setup(serial_port):
    ''' configure the Teraranger upon setup and throw an error if not connected'''
    global tera

    tera = serial.Serial(serial_port, baudrate=115200, timeout=1)

    # More information on serial commands from the documentation:
    # https://terabee.b-cdn.net/wp-content/uploads/2021/03/TeraRanger-Hub-Evo-User-Manual.pdf

    # Set data-out type to 
    set_printout_text_command = bytes.fromhex("00110145")
    tera.write(set_printout_text_command)

    # Set data update rate to 50Hz
    set_data_refresh_rate_command = bytes.fromhex("00520302C3")
    tera.write(set_data_refresh_rate_command)

    # Enable data stream from the Teraranger
    activate_stream_command = bytes.fromhex("00520201DF")
    tera.write(activate_stream_command)

    while(True):
        try:
            raw_data = tera.readline().decode()[:-2]
        except:
            print("error: could not read or decode teraranger data")
            continue
        try:
            data = raw_data.split('\t')
            front = data[1]
            left = data[7]
            right = data[3]
            data_formatted = [left, front, right]
            teraranger_data.put(data_formatted)
        except:
            print("error: could not parse teraranger data")
            continue


def telemetry_setup(serial_port):
    '''configure the Microhard P900 radio'''
    pass

    global telemetry_received_data, telemetry_send_data
    
    global rad
    rad = serial.Serial(serial_port, baudrate=57600)

    # manage the sending and receiving of byte data
    while(True):
        telemetry_received_data.put(rad.readline())
        #rad.write(telemetry_send_data.get_nowait().encode('utf-8'))


def depth_sensing_setup():
    ''' configure the OakD depth-sensing camera'''
    pass

    global oakd
    
    # Connect to device and setup pipeline
    oakd = dai.Device(create_pipeline(), usb2Mode=True)

def create_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    objectTracker = pipeline.create(dai.node.ObjectTracker)

    xoutRgb = pipeline.create(dai.node.XLinkOut)
    trackerOut = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("preview")
    trackerOut.setStreamName("tracklets")

    # Properties
    camRgb.setPreviewSize(544, 320)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # setting node configs
    stereo.initialConfig.setConfidenceThreshold(190)
    lrcheck = True
    subpixel = True

    stereo.setLeftRightCheck(lrcheck)
    stereo.setSubpixel(subpixel)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    
    # fix warning
    stereo.setDepthAlign(dai.CameraBoardSocket.LEFT)

    spatialDetectionNetwork.setBlobPath(str("/home/pi/senior-design-2023/landing_target.blob"))
    spatialDetectionNetwork.setConfidenceThreshold(0.4)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(50)
    spatialDetectionNetwork.setDepthUpperThreshold(10000)

    # objectTracker.setDetectionLabelsToTrack([15])  # track only person
    # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS
    objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_IMAGELESS)
    # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
    objectTracker.setMaxObjectsToTrack(4)

    # Linking
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    manip = pipeline.create(dai.node.ImageManip)
    manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    manip.initialConfig.setResize(300, 300)
    monoLeft.out.link(manip.inputImage)

    manip.out.link(spatialDetectionNetwork.input)
    objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
    objectTracker.out.link(trackerOut.input)

    manip.out.link(objectTracker.inputTrackerFrame)
    objectTracker.inputTrackerFrame.setBlocking(False)
    # do not block the pipeline if it's too slow on full frame
    objectTracker.inputTrackerFrame.setQueueSize(2)

    spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
    spatialDetectionNetwork.out.link(objectTracker.inputDetections)
    stereo.depth.link(spatialDetectionNetwork.inputDepth)

    # Send tracklets via SPI to the MCU
    spiOut = pipeline.create(dai.node.SPIOut)
    spiOut.setStreamName("tracklets")
    spiOut.setBusId(0)
    spiOut.input.setBlocking(False)
    spiOut.input.setQueueSize(4)
    objectTracker.out.link(spiOut.input)

    return pipeline


