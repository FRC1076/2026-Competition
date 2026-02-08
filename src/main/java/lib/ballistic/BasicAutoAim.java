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
