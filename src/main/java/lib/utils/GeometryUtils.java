// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.utils;

import org.apache.commons.lang3.NotImplementedException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class GeometryUtils {
    private GeometryUtils() {
        throw new NotImplementedException("This is a utility class!");
    }

    /** rotates a Pose2d */
    public static Pose2d rotatePose(Pose2d pose, Rotation2d rot) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().rotateBy(rot));
    }

    /* finds angle from one pose to another pose */
    public static Rotation2d angleToPose(Pose2d startPose, Pose2d endPose){
        return endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    }

    /** Creates a robot-relative ChassisSpeeds from two poses */
    public static ChassisSpeeds twoPosesToChassisSpeeds(Pose2d newPose, Pose2d oldPose, double timeSecs) {
        Twist2d twist = getVelocityBetweenPoses(newPose, oldPose, timeSecs);

        return new ChassisSpeeds(
            twist.dx,
            twist.dy,
            twist.dtheta
        );
    }

    /** Gets the velocity between two poses */
    public static Twist2d getVelocityBetweenPoses(Pose2d newPose, Pose2d oldPose, double timeDelta) {
        if (timeDelta < 1e-6) {
            return new Twist2d();
        }

        Twist2d deltaTwist = oldPose.log(newPose);

        return new Twist2d(
            deltaTwist.dx / timeDelta,
            deltaTwist.dy / timeDelta,
            deltaTwist.dtheta / timeDelta
        );
    }
}