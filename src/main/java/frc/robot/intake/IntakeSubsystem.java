// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.Map;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.ShuffleboardTabConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    private CANSparkFlex leftPivotMotor = new CANSparkFlex(IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex rightPivotMotor = new CANSparkFlex(IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    /** Vortex */
    private CANSparkFlex topIntakeMotor = new CANSparkFlex(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    /** Neo */
    private CANSparkMax bottomIntakeMotor = new CANSparkMax(IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    /** Through bore encoder */
    private RelativeEncoder pivotEncoder = bottomIntakeMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    // Shuffleboard
    private GenericEntry SB_D_PIVOT_POSITION = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
        .add("Intake Pivot", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(9, 0)
        .withSize(2, 1)
        .getEntry();

    public IntakeSubsystem() {
        super("IntakeSubsystem");
        leftPivotMotor.setInverted(false);
        rightPivotMotor.follow(leftPivotMotor, true);
        topIntakeMotor.setInverted(true);
        bottomIntakeMotor.follow(topIntakeMotor, true);
        pivotEncoder.setInverted(true);

        this.pivotEncoder.setPosition(Units.degreesToRotations(IntakeConstants.IntakeState.IDLE.getAngle()));

        // Shuffleboard layout to store Intake commands
        ShuffleboardLayout pivotList = Shuffleboard.getTab(ShuffleboardTabConstants.PITTING)
            .getLayout("Intake Pivot", BuiltInLayouts.kList)
            .withProperties(Map.of("Label position", "TOP"))
            .withPosition(3, 0)
            .withSize(3, 6);
        // Reset the pivot's position
        pivotList.add("Reset Position Lower Limit",
            Commands.runOnce(() -> {
                IntakeSubsystem.getInstance().resetPivotPosition(0);
            }).ignoringDisable(true).withName(0 + " deg"))
            .withPosition(0, 1)
            .withWidget(BuiltInWidgets.kCommand);
        pivotList.add("Reset Position Higher Limit",
            Commands.runOnce(() -> {
                IntakeSubsystem.getInstance().resetPivotPosition(IntakeConstants.IntakeState.IDLE.getAngle());
            }).ignoringDisable(true).withName(IntakeConstants.IntakeState.IDLE.getAngle() + " deg"))
            .withPosition(0, 2)
            .withWidget(BuiltInWidgets.kCommand);
    }

    /**
     * Moves the note forward through the intake
     * 
     * @param intakeSpeed the speed of the intake motors
     */
    public void setIntakeSpeed(double intakeSpeed) {
        topIntakeMotor.set(intakeSpeed);
    }

    /**
     * Set pivot motors to a specific speed
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        leftPivotMotor.set(speed);
    }

    /**
     * Gets the position of the through bore encoder
     * 
     * @return position of the intake in degrees. 0 is at hard stop when extended.
     */
    public double getPivotPosition() {
        return Units.rotationsToDegrees(this.pivotEncoder.getPosition());
    }

    /**
     * Sets the position of the through bore encoder
     * 
     * @return position of the intake in degrees. 0 is at hard stop when extended.
     */
    public void resetPivotPosition(double position) {
        this.pivotEncoder.setPosition(Units.degreesToRotations(position));
    }

    @Override
    public void periodic() {
        this.SB_D_PIVOT_POSITION.setString(PhysicalConstants.DEC_FORMAT.format(getPivotPosition()));
    }
}
