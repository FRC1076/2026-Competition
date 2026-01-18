package lib.vision.localization;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import lib.vision.AprilTagInputsAutoLogged;
import lib.vision.NorthstarCamera;

public class NorthstarLocalizer implements CameraLocalizer {
    private final NorthstarCamera camera;

    private AprilTagFieldLayout aprilTagLayout;

    private double fieldBorderMargin;
    
    private double ambiguityThreshold;

    private double xyStdDevCoefficient;
    private double thetaStdDevCoefficient;


    public NorthstarLocalizer(
        NorthstarCamera camera,
        AprilTagFieldLayout aprilTagLayout,
        double ambiguityThreshold, 
        double fieldBorderMargin,
        double xyStdDevCoefficient,
        double thetaStdDevCoefficient
    ) {
        this.camera = camera;

        this.aprilTagLayout = aprilTagLayout;

        this.fieldBorderMargin = fieldBorderMargin;

        this.ambiguityThreshold = ambiguityThreshold;

        this.xyStdDevCoefficient = xyStdDevCoefficient;
        this.thetaStdDevCoefficient = thetaStdDevCoefficient;
    }
    
    public static record NorthstarPoseEstimate (
        CommonPoseEstimate poseEstimate,
        double reciprocalStdDevs
    ) {}

    @Override
    public Optional<CommonPoseEstimate> getPoseEstimate() {
        camera.updateInputs();
        AprilTagInputsAutoLogged aprilTagInputs = camera.getAprilTagInputs();

        ArrayList<NorthstarPoseEstimate> allPoseEstimates = new ArrayList<>();

        for (int frameIndex = 0;
            frameIndex < aprilTagInputs.timestamps.length;
            frameIndex++
        ) {
            Double timestamp = aprilTagInputs.timestamps[frameIndex];
            double[] values = aprilTagInputs.frames[frameIndex];
            Optional<Pose3d> robotToCamera = camera.getConfig().poseFunction().apply(timestamp);

            // Exit if blank frame
            if (values.length == 0 || values[0] == 0 || robotToCamera.isEmpty()) {
                continue;
            }

            // Switch based on number of poses
            Pose3d cameraPose = null;
            Pose3d robotPose = null;
            boolean useVisionRotation = false;
            if (values[0] == 1) {
                // One pose (multi-tag), use directly
                cameraPose =
                    new Pose3d(
                        values[2],
                        values[3],
                        values[4],
                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                robotPose = cameraPose.transformBy(robotToCamera.get().minus(new Pose3d()).inverse());
                useVisionRotation = true;
            } else if (values[0] == 2) {
                // Two poses (one tag), disambiguate
                double error0 = values[1];
                double error1 = values[9];
                Pose3d cameraPose0 =
                    new Pose3d(
                        values[2],
                        values[3],
                        values[4],
                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                Pose3d cameraPose1 =
                    new Pose3d(
                        values[10],
                        values[11],
                        values[12],
                        new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
                Transform3d cameraToRobot = robotToCamera.get().minus(new Pose3d()).inverse();
                Pose3d robotPose0 = cameraPose0.transformBy(cameraToRobot);
                Pose3d robotPose1 = cameraPose1.transformBy(cameraToRobot);

                // Check for ambiguity and select based on estimated rotation
                if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
                    Rotation2d currentRotation = RobotState.getInstance().getRotation();
                    Rotation2d visionRotation0 = robotPose0.toPose2d().getRotation();
                    Rotation2d visionRotation1 = robotPose1.toPose2d().getRotation();
                    if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                        < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                        cameraPose = cameraPose0;
                        robotPose = robotPose0;
                    } else {
                        cameraPose = cameraPose1;
                        robotPose = robotPose1;
                    }
                }
            } else {
                continue;
            }

            // Exit if robot pose is off the field
            if (robotPose.getX() < -fieldBorderMargin
                || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
                || robotPose.getY() < -fieldBorderMargin
                || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
            continue;
            }

            // Get tag poses and update last detection times
            List<Pose3d> tagPoses = new ArrayList<>();
            for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
                Optional<Pose3d> tagPose =
                    aprilTagLayout.getTagPose((int) values[i]);
                tagPose.ifPresent(tagPoses::add);
            }
            if (tagPoses.isEmpty()) continue;

            // Calculate average distance to tag
            double totalDistance = 0.0;
            for (Pose3d tagPose : tagPoses) {
                totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
            }
            double avgDistance = totalDistance / tagPoses.size();

            // Add observation to list
            double xyStdDev =
                xyStdDevCoefficient
                    * Math.pow(avgDistance, 1.2)
                    / Math.pow(tagPoses.size(), 2.0)
                    * camera.getConfig().stdDevFactor();
            double thetaStdDev =
                useVisionRotation
                    ? thetaStdDevCoefficient
                        * Math.pow(avgDistance, 1.2)
                        / Math.pow(tagPoses.size(), 2.0)
                        * camera.getConfig().stdDevFactor()
                    : Double.POSITIVE_INFINITY;
            
            allPoseEstimates.add(
                new NorthstarPoseEstimate(
                    new CommonPoseEstimate(
                        robotPose.toPose2d(), 
                        timestamp, 
                        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev),
                        values[0] == 1
                            ? true
                            : false
                    ), 
                    1.0 / xyStdDev * xyStdDev * thetaStdDev
                )
            );
        }
        if (allPoseEstimates.size() == 0) {
            return Optional.empty();
        } else {
            allPoseEstimates.sort(Comparator.comparing(NorthstarPoseEstimate::reciprocalStdDevs));
            return Optional.of(allPoseEstimates.get(1).poseEstimate);
        }
    }

    @Override
    public String getName() {
        return camera.getDeviceId();
    }

    @Override
    public void log() {
        camera.logAprilTags();
    }
}
