// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDrive extends Command {

    // Instance of the swerve subsystem
    private SwerveSubsystem swerveSubsystem;
    // Instances of suppliers that will gather the inputs from the controller
    private final Supplier<Double> xSpeedFunction;
    private final Supplier<Double> ySpeedFunction;
    private final Supplier<Double> turningSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> fineControlFunction;
    private final Supplier<Boolean> forwardFunction;
    private final Supplier<Boolean> sideFunction;

    // Instances of Rate Limiters to ensure that the robot moves smoothly
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    /**
    * Creates new Swerve Drive Command
    * 
    * @param swerveSubsystem       - instance of swerve subsystem
    * @param xSpeedFunction        - function that will return the driver input for
    *                              the x direction
    * @param ySpeedFunction        - function that will return the driver input for
    *                              the y direction
    * @param turningSpeedFunction  - function that will return the driver input for
    *                              turing the robot
    * @param fieldOrientedFunction - function that will return if the driver wants
    *                              to be field Oriented or robot oriented
    */
    public SwerveDrive(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeedFunction,
        Supplier<Double> ySpeedFunction, Supplier<Double> turningSpeedFunction,
        Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> fineControlFunction, 
        Supplier<Boolean> forwardFunction, Supplier<Boolean> sideFunction) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turningSpeedFunction = turningSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.fineControlFunction = fineControlFunction;
        this.forwardFunction = forwardFunction;
        this.sideFunction = sideFunction;

        this.xLimiter = new SlewRateLimiter(Constants.SwerveKinematics.MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED);
        this.yLimiter = new SlewRateLimiter(Constants.SwerveKinematics.MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED);
        this.turningLimiter = new SlewRateLimiter(
        Constants.SwerveKinematics.MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        // Adds the swerve subsyetm to requirements to ensure that it is the only class
        // modifying its data at a single time
        this.addRequirements(this.swerveSubsystem);
    }

    @Override
    public void execute() {
        // gets the drivers input
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double turningSpeed = turningSpeedFunction.get();
        boolean fineControl = fineControlFunction.get();

        // Checks forcontroller deadband in case joysticks do not return perfectly to
        // origin
        xSpeed = Math.abs(xSpeed) > Constants.ControllerConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.ControllerConstants.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.ControllerConstants.DEADBAND ? turningSpeed : 0.0;

        // Limits the input to ensure smooth and depending on if fine control is active
        xSpeed = xLimiter.calculate(xSpeed) * Constants.SwerveKinematics.MAX_DRIVE_SPEED_METERS_PER_SECOND
            / (fineControl ? SwerveKinematics.FINE_CONTROL_DIVIDER : 1);
        ySpeed = yLimiter.calculate(ySpeed) * Constants.SwerveKinematics.MAX_DRIVE_SPEED_METERS_PER_SECOND
            / (fineControl ? SwerveKinematics.FINE_CONTROL_DIVIDER : 1);
        turningSpeed = turningLimiter.calculate(turningSpeed)
            * Constants.SwerveKinematics.MAX_DRIVE_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // Creates the chassis speeds from the driver input depending on current
        // orientation
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
            swerveSubsystem.getRotation2d());
        }
        else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        if(forwardFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, swerveSubsystem.getRotation2d());
        }
        if(sideFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 1, 0, swerveSubsystem.getRotation2d());
        }

        // Converts the chassis speeds to module states and sets them as the desired
        // ones for the modules
        swerveSubsystem.setChassisSpeeds(chassisSpeeds);
        // Ouputs the swerve system information
        swerveSubsystem.outputEncoderValues();
    }

    /**
    * Makes the swerve modules stop when the command ends or is interrupted
    * 
    * @param interrupted
    */
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    /**
    * Returns false because this command should run forever, throughout the robots
    * being enabled
    * 
    * @return boolean - always false
    */
    @Override
    public boolean isFinished() {
        return false;
    }
}
