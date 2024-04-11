// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.PhysicalConstants.SwerveKinematics;

/**
 * A command used to take input from the driver and move the robot accordingly.
 */
public class SwerveDriveCommand extends Command {
    // Instances of suppliers that will grab the inputs from the controller.
    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> turningSpeedSupplier;
    private final Supplier<Boolean> robotRelativeSupplier;
    private final Supplier<Boolean> fineControlSupplier;
    private final boolean enableDPadInput;
    private final Function<Integer, Boolean> dPadFunction;

    // Instances of rate limiters (acceleration) to ensure that the robot moves smoothly.
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    /**
     * Creates new SwerveDriveCommand.
     * @param xSpeedSupplier supplier for x speed between -1.0 and 1.0.
     * @param ySpeedSupplier supplier for y speed between -1.0 and 1.0.
     * @param turningSpeedSupplier supplier for the turning speed between -1.0 and 1.0.
     * @param robotRelativeSupplier supplier for driving robot-relative instead of field oriented.
     * @param fineControlSupplier supplier for driving at a reduced speed per {@link SwerveKinematics#FINE_CONTROL_COEFFICENT}.
     * @param enableDPadInput whether or not to get driving input from the directional pad (+ shape on controller).
     * @param dPadFunction function that takes angles (+ and ×) and returns whether or not they should move the robot that direction.
     */
    public SwerveDriveCommand(
        Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> turningSpeedSupplier,
        Supplier<Boolean> robotRelativeSupplier, Supplier<Boolean> fineControlSupplier, 
        boolean enableDPadInput, Function<Integer, Boolean> dPadFunction
    ) {
        setName("SwerveDriveCommand");

        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;
        this.robotRelativeSupplier = robotRelativeSupplier;
        this.fineControlSupplier = fineControlSupplier;
        this.enableDPadInput = enableDPadInput;
        this.dPadFunction = dPadFunction;

        this.xLimiter = new SlewRateLimiter(SwerveKinematics.DRIVE_SLEW_RATE_LIMIT);
        this.yLimiter = new SlewRateLimiter(SwerveKinematics.DRIVE_SLEW_RATE_LIMIT);
        this.turningLimiter = new SlewRateLimiter(SwerveKinematics.TURNING_SLEW_RATE_LIMIT);

        this.addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void execute() {
        // Get the driver's input from the suppliers.
        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double turningSpeed = turningSpeedSupplier.get();
        double fineControlCoefficient = fineControlSupplier.get() ? SwerveKinematics.FINE_CONTROL_COEFFICENT : 1;

        // Removes input within the deadband to prevent stick drift.
        xSpeed = Math.abs(xSpeed) > ControllerConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > ControllerConstants.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > ControllerConstants.DEADBAND ? turningSpeed : 0.0;

        // Limits the rate of change of the input depending on if fine control is active.
        xSpeed = xLimiter.calculate(xSpeed) * SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        ySpeed = yLimiter.calculate(ySpeed) * SwerveKinematics.DRIVE_SPEED_COEFFICENT;
        turningSpeed = turningLimiter.calculate(turningSpeed) * SwerveKinematics.TURNING_SPEED_COEFFIECENT;

        // Creates the chassis speeds from the driver input depending on robot or field relative.
        ChassisSpeeds chassisSpeeds;
        double[] dPadSpeeds = this.calculateDPad();
        if (this.enableDPadInput && (dPadSpeeds[0] != 0 || dPadSpeeds[1] != 0)) {
            if (robotRelativeSupplier.get()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    dPadSpeeds[0], dPadSpeeds[1], 0, SwerveSubsystem.getInstance().getRotation2d());
            }
            else {
                chassisSpeeds = new ChassisSpeeds(dPadSpeeds[0], dPadSpeeds[1], 0);
            }
        }
        else {
            if (robotRelativeSupplier.get()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, SwerveSubsystem.getInstance().getRotation2d());
            }
            else {
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }
        }

        // Multiplies by fine control to limit the speeds.
        chassisSpeeds = new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond * fineControlCoefficient,
            chassisSpeeds.vyMetersPerSecond * fineControlCoefficient,
            chassisSpeeds.omegaRadiansPerSecond * fineControlCoefficient
        );
        
        SwerveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Calculates and returns D-Pad input
     * @return the x [0] and y [1] speeds between -1.0 and 1.0
     */
    private double[] calculateDPad() {
        double[] speeds = new double[]{0, 0};
        
        // Up.
        if (dPadFunction.apply(315) || dPadFunction.apply(0) || dPadFunction.apply(45)) {
            speeds[0] = SwerveKinematics.D_PAD_SPEED;
        }
        // Down.
        else if (dPadFunction.apply(225) || dPadFunction.apply(180) || dPadFunction.apply(135)) {
            speeds[0] = -SwerveKinematics.D_PAD_SPEED;
        }
        // Left.
        if (dPadFunction.apply(225) || dPadFunction.apply(270) || dPadFunction.apply(315)) {
            speeds[1] = SwerveKinematics.D_PAD_SPEED;
        }
        // Right.
        else if (dPadFunction.apply(45) || dPadFunction.apply(90) || dPadFunction.apply(135)) {
            speeds[1] = -SwerveKinematics.D_PAD_SPEED;
        }
        return speeds;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopModules();
    }

    // This command will always never end due to being a default command.
    @Override
    public boolean isFinished() {
        return false;
    }
}
