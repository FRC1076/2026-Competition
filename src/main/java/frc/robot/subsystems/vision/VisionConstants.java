// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.Optional;
import java.util.function.Function;

public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;

  private static int monoExposure = 2200;
  private static double monoGain = 17.5;
  private static double monoDenoise = 1.0;

  public static CameraConfig[] cameras =
    new CameraConfig[] {
        CameraConfig.builder()
            .poseFunction(
                (Double timestamp) -> {
                    return Optional.of(new Pose3d());
                })
            .id("0")
            .width(1600)
            .height(1200)
            .exposure(monoExposure)
            .gain(monoGain)
            .denoise(monoDenoise)
            .stdDevFactor(1.0)
            .build()
    };

  public record CameraConfig(
    Function<Double, Optional<Pose3d>> poseFunction,
    String id,
    int width,
    int height,
    int autoExposure,
    int exposure,
    double gain,
    double denoise,
    double stdDevFactor) {

    // The static method to start the builder process
    public static CameraConfigBuilder builder() {
        return new CameraConfigBuilder();
    }

    // The Inner Builder Class - WRITTEN BY GEMINI
    public static class CameraConfigBuilder {
        private Function<Double, Optional<Pose3d>> poseFunction;
        private String id;
        private int width;
        private int height;
        private int autoExposure;
        private int exposure;
        private double gain;
        private double denoise;
        private double stdDevFactor;

        CameraConfigBuilder() {}

        public CameraConfigBuilder poseFunction(Function<Double, Optional<Pose3d>> poseFunction) {
            this.poseFunction = poseFunction;
            return this;
        }

        public CameraConfigBuilder id(String id) {
            this.id = id;
            return this;
        }

        public CameraConfigBuilder width(int width) {
            this.width = width;
            return this;
        }

        public CameraConfigBuilder height(int height) {
            this.height = height;
            return this;
        }

        public CameraConfigBuilder autoExposure(int autoExposure) {
            this.autoExposure = autoExposure;
            return this;
        }

        public CameraConfigBuilder exposure(int exposure) {
            this.exposure = exposure;
            return this;
        }

        public CameraConfigBuilder gain(double gain) {
            this.gain = gain;
            return this;
        }

        public CameraConfigBuilder denoise(double denoise) {
            this.denoise = denoise;
            return this;
        }

        public CameraConfigBuilder stdDevFactor(double stdDevFactor) {
            this.stdDevFactor = stdDevFactor;
            return this;
        }

        // The terminal method that calls the Record's constructor
        public CameraConfig build() {
            return new CameraConfig(
                poseFunction, id, width, height, 
                autoExposure, exposure, gain, denoise, stdDevFactor
            );
        }

        public String toString() {
            return "CameraConfig.CameraConfigBuilder(id=" + this.id + ")";
        }
    }
}

  private VisionConstants() {}
}
