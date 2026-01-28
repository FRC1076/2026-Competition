package lib.ballistic;

import edu.wpi.first.math.geometry.Pose3d;

public record CommonSOTMSolution (
    Pose3d effectiveTarget,
    double launchPitchRad,
    double launchSpeed,
    double flightTimeSeconds,
    double launchYawRad
) { }
