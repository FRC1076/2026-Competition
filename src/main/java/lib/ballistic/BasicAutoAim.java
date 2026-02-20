// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.ballistic;

import edu.wpi.first.math.geometry.Pose3d;

public class BasicAutoAim {
    public static CommonShotSolution calculate(
        Pose3d shooterPose,
        Pose3d targetPose
    ) {
        Pose3d shooterRelativeTarget = targetPose.relativeTo(shooterPose);
        return new CommonShotSolution(
            shooterRelativeTarget.getRotation().getZ(), shooterRelativeTarget.getY(), 0);
    }
}
