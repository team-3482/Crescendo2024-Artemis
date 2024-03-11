// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.ShuffleboardTabConstants;
import frc.robot.utilities.JSONManager;

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

    /** Check for saving position to roborio */
    private boolean savedPosition = false;

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

        double position = JSONManager.getInstance().getIntakePivotPosition();
        // double position = 0;
        // JSONManager.getInstance().saveIntakePivotPosition(position);
        this.pivotEncoder.setPosition(Units.degreesToRotations(position));
        JSONManager.getInstance().saveIntakePivotPosition(position);
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
     * Set pivot motors to a specific rotation (degrees)
     * 
     * @param speed between -1.0 and 1.0
     */
    public void setPivotSpeed(double speed) {
        leftPivotMotor.set(speed);
    }

    /**
     * Gets the position of the through bore encoder
     * 
     * @return position of the intake in degrees. 0 is at hardware stop when extended.
     */
    public double getPivotPosition() {
        return Units.rotationsToDegrees(this.pivotEncoder.getPosition());
    }

    @Override
    public void periodic() {
        double position = getPivotPosition();
        this.SB_D_PIVOT_POSITION.setString(PhysicalConstants.DEC_FORMAT.format(position));

        if (this.pivotEncoder.getVelocity() != 0) {
            this.savedPosition = false;
        }
        else if (!this.savedPosition) {
            if ((int) position == 0) {
                position = 0;
                this.pivotEncoder.setPosition(position);
            }
            JSONManager.getInstance().saveIntakePivotPosition(position);
            this.savedPosition = true;
        }
    }
}
