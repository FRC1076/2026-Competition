package lib.vision;

import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose3d;

public class NorthstarCameraConfig {
    Function<Double, Optional<Pose3d>> poseFunction;
    String id;
    int width;
    int height;
    int autoExposure;
    int exposure;
    double gain;
    double denoise;
    double stdDevFactor;

    public NorthstarCameraConfig(
        Function<Double, Optional<Pose3d>> poseFunction,
        String id,
        int width,
        int height,
        int autoExposure,
        int exposure,
        double gain,
        double denoise,
        double stdDevFactor
    ) {
        this.poseFunction = poseFunction;
        this.id = id;
        this.width = width;
        this.height = height;
        this.autoExposure = autoExposure;
        this.exposure = exposure;
        this.gain = gain;
        this.denoise = denoise;
        this.stdDevFactor = stdDevFactor;
    }

    public Function<Double, Optional<Pose3d>> poseFunction() {
        return poseFunction;
    }

    public String id() {
        return id;
    }

    public int width() {
        return width;
    }

    public int height() {
        return height;
    }

    public int autoExposure() {
        return autoExposure;
    }

    public int exposure() {
        return exposure;
    }

    public double gain() {
        return gain;
    }

    public double denoise() {
        return denoise;
    }

    public double stdDevFactor() {
        return stdDevFactor;
    }
}
