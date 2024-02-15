// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OrbitConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveOrbit extends Command {
    // Instance of the subsystems
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    // Instances of suppliers that will gather the inputs from the controller
    private final Supplier<Double> xSpeedFunction;
    private final Supplier<Double> ySpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> fineControlFunction;
    private final boolean enableDPadInput;
    private final Function<Integer, Boolean> povFunction;

    // Instances of Rate Limiters to ensure that the robot moves smoothly
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;
    private PIDController rotationPidController;

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
    public SwerveOrbit(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem,
        Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction,
        Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> fineControlFunction,
        boolean enableDPadInput, Function<Integer, Boolean> povFunction) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.fineControlFunction = fineControlFunction;
        this.enableDPadInput = enableDPadInput;
        this.povFunction = povFunction;

        this.xLimiter = new SlewRateLimiter(OrbitConstants.ORBIT_DRIVE_SLEW_RATE_LIMIT);
        this.yLimiter = new SlewRateLimiter(OrbitConstants.ORBIT_DRIVE_SLEW_RATE_LIMIT);
        this.turningLimiter = new SlewRateLimiter(OrbitConstants.ORBIT_TURNING_SLEW_RATE_LIMIT);
        this.rotationPidController = new PIDController(
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KP,
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KI,
            OrbitConstants.TURNING_SPEED_PID_CONTROLLER.KD);
        
        // Adds the swerve subsyetm to requirements to ensure that it is the only class
        // modifying its data at a single time
        // Do not require limelight subsystem because it is getter functions only
        this.addRequirements(this.swerveSubsystem); 
    }

    @Override
    public void execute() {
        // Double so it can be null if the ID cannot be orbited
        Double orbitOffset = OrbitConstants.ORBIT_IDS.get(limelightSubsystem.getID()); 
        // If it isn't tags 3 or 4 or their respective blue tags
        if (orbitOffset == null) {
            swerveSubsystem.stopModules();
            return;
        }
        // Gets the driver's input
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        boolean fineControl = fineControlFunction.get();

        // Orbit calculations
        double[] botPose_TargetSpace = limelightSubsystem.getBotPose_TargetSpace();
        double angleGoalRad = Math.atan2(botPose_TargetSpace[0], -botPose_TargetSpace[2]); // +/- orbitOffset for [0]
        double turningSpeed = rotationPidController
            .calculate(swerveSubsystem.getHeading(), Units.radiansToDegrees(angleGoalRad)) / 60 * 1.1;

        // Checks for controller deadband in case joysticks do not return perfectly to origin
        xSpeed = Math.abs(xSpeed) > ControllerConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > ControllerConstants.DEADBAND ? ySpeed : 0.0;

        // Limits the input to ensure smooth and depending on if fine control is active
        xSpeed = xLimiter.calculate(xSpeed) * SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        ySpeed = yLimiter.calculate(ySpeed) * SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        turningSpeed = turningLimiter.calculate(turningSpeed)
            * SwerveKinematics.TURNING_SPEED_COEFFIECENT;
        
        // Creates the chassis speeds from the driver input depending on current orientation
        ChassisSpeeds chassisSpeeds;
        double[] dPadSpeeds = this.calculateDPad();
        if (this.enableDPadInput && (dPadSpeeds[0] != 0 || dPadSpeeds[1] != 0)) {
            if (fieldOrientedFunction.get()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    dPadSpeeds[0], dPadSpeeds[1], 0, swerveSubsystem.getRotation2d());
            }
            else {
                chassisSpeeds = new ChassisSpeeds(dPadSpeeds[0], dPadSpeeds[1], 0);
            }
        }
        else {
            if (fieldOrientedFunction.get()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            }
            else {
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }
        }
        // Multiplies by fine control to limit the speeds
        double fineControlCoefficent = fineControl ? OrbitConstants.ORBIT_FINE_CONTROL_SPEED_COEFFIECENT : OrbitConstants.ORBIT_SPEED_COEFFIECENT;
        chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond * fineControlCoefficent, 
                                            chassisSpeeds.vyMetersPerSecond * fineControlCoefficent, 
                                            chassisSpeeds.omegaRadiansPerSecond * fineControlCoefficent);
        
        // Converts the chassis speeds to module states and sets them as the desired
        // ones for the modules
        swerveSubsystem.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Calculates and returns D-Pad input
     *
     * @return the x and y speeds between -1 and 1
     */
    public double[] calculateDPad() {
        double[] speeds = new double[]{0, 0};
        
        // Up
        if (povFunction.apply(315) || povFunction.apply(0) || povFunction.apply(45)) {
            speeds[0] = SwerveKinematics.D_PAD_SPEED;
        }
        // Down
        else if (povFunction.apply(225) || povFunction.apply(180) || povFunction.apply(135)) {
            speeds[0] = -SwerveKinematics.D_PAD_SPEED;
        }
        // Left
        if (povFunction.apply(225) || povFunction.apply(270) || povFunction.apply(315)) {
            speeds[1] = SwerveKinematics.D_PAD_SPEED;
        }
        // Right
        else if (povFunction.apply(45) || povFunction.apply(90) || povFunction.apply(135)) {
            speeds[1] = -SwerveKinematics.D_PAD_SPEED;
        }
        return speeds;
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
    * Returns false because this command should run until the user releases the button
    * 
    * @return boolean - always false
    */
    @Override
    public boolean isFinished() {
        return false;
    }
}
