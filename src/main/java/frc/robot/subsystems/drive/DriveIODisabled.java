package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Shell class for running the robot without a physical drivetrain */
public class DriveIODisabled implements DriveIO {
    public void updateInputs(DriveIOInputs inputs) {}
    public void acceptRequest(SwerveRequest request) {}
    public void updateModuleInputs(ModuleIOInputs inputs, int moduleIndex) {}
    public Translation2d[] getModuleLocations() {return new Translation2d[] {};}
    public void addVisionMeasurement(Pose2d poseEstimate, double timestampSeconds, Matrix<N3, N1> StdDevs) {}
    public void resetPose(Pose2d pose) {}
    public void resetHeading() {}
    public Pose2d getPose() {return new Pose2d();}
    public void setAllianceRotation(Rotation2d allianceRotation) {}
    public void periodic() {}
    public void setDriveStatorCurrentLimit(double currentLimit) {}
}
