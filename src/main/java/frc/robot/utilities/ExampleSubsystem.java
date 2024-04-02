// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern
    private static ExampleSubsystem instance;
    public static ExampleSubsystem getInstance() {
        if(instance == null) {
            instance = new ExampleSubsystem();
        }
        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    public ExampleSubsystem() {
        super("ExampleSubsystem");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
