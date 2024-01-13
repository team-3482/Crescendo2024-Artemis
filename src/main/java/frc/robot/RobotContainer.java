// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    // Singleton design pattern
    public static RobotContainer instance;

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

    // Instance of the Swerve Subsystem
    private SwerveSubsystem swerveSubsystem;
    // Instance of the controller used to drive the robot
    private CommandXboxController driveController;

    /**
    * Creates an instance of the robot controller
    */
    public RobotContainer() {
        this.swerveSubsystem = new SwerveSubsystem();
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
            ()-> driveController.getHID().getXButton(), 
            ()-> driveController.getHID().getBButton()));

        configureBindings();
    }

    /**
    * Configures the button bindings of the controllers
    */
    private void configureBindings() {
        driveController.y().whileTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));
    }
  
    // public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config =
    //   new TrajectoryConfig(
    //     SwerveKinematics.MAX_DRIVE_SPEED_METERS_PER_SECOND,
    //     SwerveKinematics.MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(SwerveKinematics.driveKinematics);
    //
    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory =
    //   TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);
    //
    // var thetaController =
    //   new ProfiledPIDController(
    //     SwerveKinematics.AUTO_PID_THETA_CONTROLLER_KP, 0, 0, SwerveKinematics.AUTO_PID_THETA_CONTROLLER_CONSTRAINTS);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //
    // SwerveControllerCommand swerveControllerCommand =
    //   new SwerveControllerCommand(
    //     exampleTrajectory,
    //     swerveSubsystem::getPose, // Functional interface to feed supplier
    //     SwerveKinematics.driveKinematics,
    //
    //     // Position controllers
    //     new PIDController(SwerveKinematics.AUTO_PID_XCONTROLLER_KP, 0, 0),
    //     new PIDController(SwerveKinematics.AUTO_PID_YCONTROLLER_KP, 0, 0),
    //     thetaController,
    //     swerveSubsystem::setModuleStates,
    //     swerveSubsystem);
    //
    // // Reset odometry to the initial pose of the trajectory, run path following
    // // command, then stop at the end.
    // return Commands.sequence(
    //     new InstantCommand(() -> swerveSubsystem.resetOdometry(exampleTrajectory.getInitialPose())),
    //     swerveControllerCommand,
    //     // new InstantCommand(() -> swerveSubsystem.drive(0, 0, 0, false)));
    //     new InstantCommand(() -> swerveSubsystem.setChasisSpeeds(new ChassisSpeeds(0, 0, 0))));
    // }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutonConstants.AUTON_PATH);
        return AutoBuilder.followPath(path);
    }
}
