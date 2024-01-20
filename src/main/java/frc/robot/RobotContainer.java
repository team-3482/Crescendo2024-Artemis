// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    // Singleton design pattern
    public static RobotContainer instance;
    private final SendableChooser<Command> autoChooser;

    /**
    * Gets the instance of the RobotContainer
    * 
    * @return instance
    */
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    // Instance of Subsystems
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    // Instance of the controller used to drive the robot
    private CommandXboxController driveController;

    /**
    * Creates an instance of the robot controller
    */
    public RobotContainer() {
        this.swerveSubsystem = new SwerveSubsystem();
        this.limelightSubsystem = new LimelightSubsystem();
        this.driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER_ID);
        
        // Sets the default command to driving swerve
        this.swerveSubsystem.setDefaultCommand(new SwerveDrive(
            swerveSubsystem,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            // () ->
            // driveController.getRawAxis(Constants.ControllerConstants.DRIVE_ROT_AXIS),
            () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis(),
            () -> !driveController.getHID().getAButton(),
            () -> driveController.getHID().getRightBumper(),
            () -> driveController.povUp().getAsBoolean(), // Replacing driveController.getHID.getXButton()
            () -> driveController.povDown().getAsBoolean()));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
    * Configures the button bindings of the controllers
    */
    private void configureBindings() {
        driveController.y().whileTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));
        driveController.x().whileTrue(Commands.run(() -> swerveSubsystem.zeroDrivePositions()));
        // Reset odometry translation to the position that the limelight sees.
        // Does not reset rotation, which is tracked by the gyro.
        driveController.a().whileTrue(Commands.run(() -> {
            Translation2d translation = limelightSubsystem.getBotpose().getTranslation();
            if (!translation.equals(new Translation2d(0, 0))) {
                swerveSubsystem.resetOdometry(new Pose2d(
                    translation, new Rotation2d(Units.degreesToRadians(swerveSubsystem.getHeading()))));
            }
        }));
        driveController.b().onTrue(new PathfindAprilTagCommand(limelightSubsystem, swerveSubsystem));
    }
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
