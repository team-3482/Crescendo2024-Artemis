// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShuffleboardTabConstants;
import frc.robot.swerve.BezierToGoalCommand;
import frc.robot.swerve.SwerveDriveCommand;
import frc.robot.swerve.SwerveOrbitCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.limelight.LimelightSubsystem;

public class RobotContainer {
    // Singleton design pattern
    private static RobotContainer instance;
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

    private final SendableChooser<Command> autoChooser;

    // Instance of the controller used to drive the robot
    private CommandXboxController driveController;

    /**
    * Creates an instance of the robot controller
    */
    public RobotContainer() {
        this.driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER_ID);
        
        // Register named commands for pathplanner (do this after subsystem initialization)
        NamedCommands.registerCommand("Pathfind AMP",
            new BezierToGoalCommand(AutonConstants.AMP));
        NamedCommands.registerCommand("Pathfind SPEAKER",
            new BezierToGoalCommand(AutonConstants.SPEAKER));

        // Sets the default command to driving swerve
        SwerveSubsystem.getInstance().setDefaultCommand(new SwerveDriveCommand(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> !(driveController.getHID().getLeftTriggerAxis() >= 0.5),
            () -> driveController.getHID().getRightTriggerAxis() >= 0.5,
            // D-Pad / POV movement
            ControllerConstants.DPAD_DRIVE_INPUT,
            (Integer angle) -> driveController.pov(angle).getAsBoolean()
        ));
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
            .add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 3)
            .withSize(3, 1);
    }

    /**
    * Configures the button bindings of the controllers
    */
    private void configureBindings() {
        // Driver controller
        // Zeroing functions
        driveController.rightBumper().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().zeroHeading()));
        // Reset odometry translation to the position that the limelight sees.
        // Does not reset rotation, which is tracked by the gyro.
        driveController.leftBumper().onTrue(Commands.runOnce(() -> {
            Translation2d translation = LimelightSubsystem.getInstance().getBotpose().getTranslation();
            if (!translation.equals(new Translation2d(0, 0))) {
                SwerveSubsystem.getInstance().resetOdometry(new Pose2d(
                    translation, Rotation2d.fromDegrees(SwerveSubsystem.getInstance().getHeading())));
            }
        }));
        // Cancel all scheduled commands
        driveController.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        // Orbit April-Tag
        driveController.a().toggleOnTrue(new SwerveOrbitCommand(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> !(driveController.getHID().getLeftTriggerAxis() >= 0.5),
            () -> driveController.getHID().getRightTriggerAxis() >= 0.5,
            // D-Pad / POV Movement
            ControllerConstants.DPAD_DRIVE_INPUT,
            (Integer angle) -> driveController.pov(angle).getAsBoolean()
            ));
        
        // Operator controller
        // Line up to AMP
        driveController.x().onTrue(new BezierToGoalCommand(AutonConstants.AMP));
        // Line up to SPEAKER
        // driveController.y().whileTrue(new PathfindLineUp(SwerveSubsystem.getInstance(), AutonConstants.SPEAKER));
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
