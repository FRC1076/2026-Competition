package lib.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class NorthstarCamera {
    private final String deviceId;
    private final NorthstarCameraConfig config;

    private final DoubleArraySubscriber observationSubscriber;
    private final DoubleArraySubscriber objDetectObservationSubscriber;
    private final IntegerSubscriber fpsAprilTagsSubscriber;
    private final IntegerSubscriber fpsObjDetectSubscriber;
    private final StringPublisher eventNamePublisher;
    private final IntegerPublisher matchTypePublisher;
    private final IntegerPublisher matchNumberPublisher;
    private final IntegerPublisher timestampPublisher;
    private final BooleanPublisher isRecordingPublisher;

    // TODO: does havingo one of these for each camera break something?
    private static final LoggedNetworkBoolean recordingRequest =
      new LoggedNetworkBoolean("/SmartDashboard/Enable Recording", false);

    private final Timer timer = new Timer();

    public boolean ntConnected = false;

    @AutoLog
    public static class AprilTagInputs {
        public double[] timestamps = new double[] {};
        public double[][] frames = new double[][] {};
        public long fps = 0;
    }

    @AutoLog
    public static class ObjectDetectionInputs {
        public double[] timestamps = new double[] {};
        public double[][] frames = new double[][] {};
        public long fps = 0;
    }

    AprilTagInputsAutoLogged aprilTagInputs = new AprilTagInputsAutoLogged();
    ObjectDetectionInputsAutoLogged odInputs = new ObjectDetectionInputsAutoLogged();

    public NorthstarCamera(NorthstarCameraConfig config, int index, String aprilTagLayout) {
        this.deviceId = "northstar_" + index;
        this.config = config;

        var northstarTable = NetworkTableInstance.getDefault().getTable(this.deviceId);
        var configTable = northstarTable.getSubTable("config");

        configTable.getStringTopic("camera_id").publish().set(config.id());
        configTable.getIntegerTopic("camera_resolution_width").publish().set(config.width());
        configTable.getIntegerTopic("camera_resolution_height").publish().set(config.height());
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(config.autoExposure());
        configTable.getIntegerTopic("camera_exposure").publish().set(config.exposure());
        configTable.getDoubleTopic("camera_gain").publish().set(config.gain());
        configTable.getDoubleTopic("camera_denoise").publish().set(config.denoise());
        configTable.getDoubleTopic("fiducial_size_m").publish().set(config.tagWidth());
        configTable.getStringTopic("tag_layout").publish().set(aprilTagLayout);
        isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();
        timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
        eventNamePublisher = configTable.getStringTopic("event_name").publish();
        matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
        matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber =
            outputTable
                .getDoubleArrayTopic("observations")
                .subscribe(
                    new double[] {},
                    PubSubOption.keepDuplicates(true),
                    PubSubOption.sendAll(true),
                    PubSubOption.pollStorage(5),
                    PubSubOption.periodic(0.01));

        objDetectObservationSubscriber =
            outputTable
                .getDoubleArrayTopic("objdetect_observations")
                .subscribe(
                    new double[] {},
                    PubSubOption.keepDuplicates(true),
                    PubSubOption.sendAll(true),
                    PubSubOption.pollStorage(5),
                    PubSubOption.periodic(0.01));

        fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
        fpsObjDetectSubscriber = outputTable.getIntegerTopic("fps_objdetect").subscribe(0);

        timer.start();

        if (DriverStation.isFMSAttached() || recordingRequest.get()) {
            isRecordingPublisher.set(true);
        } else {
            isRecordingPublisher.set(false);
        }
    }

    public void updateInputs() {
        // Check if this camera is active
        ntConnected = false;
        for (var client : NetworkTableInstance.getDefault().getConnections()) {
            if (client.remote_id.startsWith(this.deviceId)) {
                ntConnected = true;
                break;
            }
        }

        boolean slowPeriodic = timer.advanceIfElapsed(1.0);

        // Publish timestamp
        if (slowPeriodic) {
            timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);
            eventNamePublisher.set(DriverStation.getEventName());
            matchTypePublisher.set(DriverStation.getMatchType().ordinal());
            matchNumberPublisher.set(DriverStation.getMatchNumber());
        }

        // Get AprilTag data
        var aprilTagQueue = observationSubscriber.readQueue();
        aprilTagInputs.timestamps = new double[aprilTagQueue.length];
        aprilTagInputs.frames = new double[aprilTagQueue.length][];
        for (int i = 0; i < aprilTagQueue.length; i++) {
            aprilTagInputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
            aprilTagInputs.frames[i] = aprilTagQueue[i].value;
        }
        if (slowPeriodic) {
            aprilTagInputs.fps = fpsAprilTagsSubscriber.get();
        }

        // Get object detection data
        var objDetectQueue = objDetectObservationSubscriber.readQueue();
        odInputs.timestamps = new double[objDetectQueue.length];
        odInputs.frames = new double[objDetectQueue.length][];
        for (int i = 0; i < objDetectQueue.length; i++) {
            odInputs.timestamps[i] = objDetectQueue[i].timestamp / 1000000.0;
            odInputs.frames[i] = objDetectQueue[i].value;
        }
        if (slowPeriodic) {
            odInputs.fps = fpsObjDetectSubscriber.get();
        }
    }

    public void setRecording(boolean active) {
        isRecordingPublisher.set(active);
    }

    public String getDeviceId() {
        return deviceId;
    }

    public void logAprilTags() {
        Logger.processInputs("Vision/AprilTags/" + deviceId, aprilTagInputs);
    }

    public void logObjDetection() {
        Logger.processInputs("Vision/ObjDetection/" + deviceId, odInputs);
    }

    public AprilTagInputsAutoLogged getAprilTagInputs() {
        return aprilTagInputs;
    }

    public ObjectDetectionInputsAutoLogged getObjDetectionInputs() {
        return odInputs;
    }

    public NorthstarCameraConfig getConfig() {
        return config;
    }
}
