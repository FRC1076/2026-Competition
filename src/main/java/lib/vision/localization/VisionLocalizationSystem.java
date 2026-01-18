// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.vision.localization;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.functional.TriConsumer;


/**
 * A class that manages robot position estimates from multiple cameras and 
 * sends them to all externally registered consumers using the {@link VisionLocalizationSystem#update()} method
 * <p>Consumers are registered using the {@link VisionLocalizationSystem#registerMeasurementConsumer()} method
 */
public class VisionLocalizationSystem {

    private class CamStruct {
        public final CameraLocalizer camera;
        public boolean cameraActive = true; //Signals whether or not the camera reading should be added to vision measurements

        /** Sets whether or not the camera is active */ 
        public void setActive(boolean active) {
            cameraActive = active;
        }

        public CamStruct(CameraLocalizer camera) {
            this.camera = camera;
        }
    }

    //A Consumer that accepts a Pose3d and a Matrix of Standard Deviations, usually should call addVisionMeasurements() on a SwerveDrivePoseEstimator3d
    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> measurementConsumer = (pose, timestamp, stddevs) -> {}; //A default no-op consumer is instantiated to prevent null pointer dereferences

    private final Map<String, CamStruct> cameras = new HashMap<>();

    private double fieldWidthMeters = 8.07;
    private double fieldLengthMeters = 16.54;
    private double fieldPerimeterToleranceMeters = 0.5;
    private boolean validateForFieldPerimeter = true;
    
    /**
     * Registers a consumer
     * <p> This method should be called externally on initialization
     * 
     * @param consumer A consumer that accepts a Pose2d, Double, and 3x1 Matrix
     * <p> The Pose2d is the robot pose estimate in meters
     * <p> The Double is the timestamp of the measurement in seconds
     * <p> The 3x1 Matrix is the standard deviations of the measurement
     */
    public void registerMeasurementConsumer(TriConsumer<Pose2d, Double, Matrix<N3, N1>> consumer) {
        measurementConsumer = measurementConsumer.andThen(consumer);
    }

    /**
     * Adds a camera to the vision system
     */
    public void addCamera(CameraLocalizer camera) {
        CamStruct camStruct = new CamStruct(camera);
        cameras.put(camera.getName(), camStruct);
    }

    /**
     * Enables or disables selected cameras
     * @param enabled Whether or not the cameras should be enabled
     * @param cams The IDs of the cameras to enable or disable
     */
    public void enableCameras(boolean enabled, String... cams) {
        for (String camID : cams) {
            cameras.get(camID).setActive(enabled);
        }
    }

    /**
     * Enables or disables all cameras
     * @param enabled Whether or not the cameras should be enabled
     */
    public void enableAllCameras(boolean enabled) {
        for (var camStruct : cameras.values()) {
            camStruct.setActive(enabled);
        }
    }
    
    /**
     * For PhotonVision cameras, this method will set the pose estimator strategy from {@link PhotonPoseEstimator#PoseStrategy}
     * For Limelight cameras, this method will call the default method, which does nothing
     */
    public void setPhotonPoseStrategy(PhotonPoseEstimator.PoseStrategy strategy, String... cams){
        for (String camID : cams) {
            cameras.get(camID).camera.setPoseStrategy(strategy);
        }
    }

    /** Set the width of the field perimeter for validation of poses */
    public void setFieldWidth(double fieldWidthMeters) {
        this.fieldWidthMeters = fieldWidthMeters;
    }

    /** Set the length of the field perimeter for validation of poses */
    public void setFieldLength(double fieldLengthMeters) {
        this.fieldLengthMeters = fieldLengthMeters;
    }

    /** Set the tolerance with the perimeter of the field for validating poses */
    public void setValidationTolerance(double toleranceMeters) {
        this.fieldPerimeterToleranceMeters = toleranceMeters;
    }

    /** Set whether or not to validate poses for being within the field perimeter */
    public void enablePoseValidation(boolean enable) {
        this.validateForFieldPerimeter = enable;
    }

    /**
     * Fetches pose estimate from cameras and sends them to all consumers. This function should be called exactly once every main function loop
     */
    public void update() {
        for (var camStruct : cameras.values()) {

            camStruct.camera.log();

            if (camStruct.cameraActive) {
                camStruct.camera.getPoseEstimate().ifPresent(
                    (estimate) -> {
                        if ((estimate.pose().getX() > -fieldPerimeterToleranceMeters
                            && estimate.pose().getX() < fieldLengthMeters + fieldPerimeterToleranceMeters
                            && estimate.pose().getY() > -fieldPerimeterToleranceMeters
                            && estimate.pose().getY() < fieldWidthMeters + fieldPerimeterToleranceMeters)
                            || !validateForFieldPerimeter) {
                            measurementConsumer.accept(
                                estimate.pose(),
                                estimate.timestampSeconds(),
                                estimate.stdDevs()
                            );
                        }
                    }
                );
            }
        }
    }
}