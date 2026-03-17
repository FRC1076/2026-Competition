// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.AutonSides;

public class Elastic {
    // private SendableChooser<TeamColors> teamChooser;
    private Field2d field;
    private SendableChooser<AutonSides> autonSideChooser;
    private SendableChooser<Alliance> allianceChooser;
    private SendableChooser<Command> autoChooser;

    public Elastic() {
        /* This is a dropdown menu on the SmartDashboard that allows the user to select whether 
        the auton is on the left (default) or the right side of the field.
        */
        field = new Field2d();
        SmartDashboard.putData(field);

        autonSideChooser = new SendableChooser<>();
        autonSideChooser.setDefaultOption(GameConstants.autonSide.name(), GameConstants.autonSide);
        autonSideChooser.addOption(AutonSides.Left.name(), AutonSides.Left);
        autonSideChooser.addOption(AutonSides.Right.name(), AutonSides.Right);
        SmartDashboard.putData("Auton Side Chooser", autonSideChooser);

        // Allow selection of alliance on Elastic
        allianceChooser = new SendableChooser<>();
        allianceChooser.setDefaultOption(GameConstants.teamColor.name(), GameConstants.teamColor);
        allianceChooser.addOption(Alliance.Blue.name(), Alliance.Blue);
        allianceChooser.addOption(Alliance.Red.name(), Alliance.Red);
        SmartDashboard.putData("Team Color", allianceChooser);

        // Init auto chooser to be empty
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("None", Commands.none());
        
        // Initialize fields, because otherwise they're only updated when teleop is enabled
        this.putNumber("FlywheelVelocityTarget", 0);
    }

    public void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public double readNumber(String key) {
        return SmartDashboard.getNumber(key, 0);
    }

    public void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public boolean readBoolean(String key) {
        return SmartDashboard.getBoolean(key, false);
    }

    public void putString(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    /** Gets the selected team color from the driver station */
    public Alliance getSelectedTeamColor() {
        return allianceChooser.getSelected();
    }

    public void updateField(Pose2d robotPose) {
        field.setRobotPose(robotPose);
    }

    /** Returns true to mirror the auton from the left side to the right side
     * when in autonomous mode and the auton is selected as mirrored to the right side */
    public boolean getPathPlannerMirrored() {
        return DriverStation.isAutonomous() && autonSideChooser.getSelected().isRightSide; //(GameConstants.autonSide == AutonSides.Right);
    }

    /** Build the auto chooser and send it to Elastic after the AutoBuilder has been configured. */
    public void buildAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /** Returns the selected auto from the auto chooser. */
    public Command getSelectedAutonomousCommand() {
        return autoChooser.getSelected();
    }
}