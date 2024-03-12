// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.utilities.Telemetry;
import frc.robot.utilities.JSONManager;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command auton;
    /**
    * This function is run when the robot is first started up and should be used
    * for any initialization code.
    */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        RobotContainer.getInstance();

        // Port forward all required LL ports. Necessary for robot connections over ethernet (eg. at comp)
        for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, LimelightConstants.INTAKE_LLIGHT + ".local", port);
            PortForwarder.add(port, LimelightConstants.SHOOTER_LLIGHT + ".local", port);
        }

        // Start telemetry
        Telemetry.getInstance();
        addPeriodic(() -> Telemetry.getInstance().publish(), PhysicalConstants.TELEMETRY_LOOP_TIME);
    }

    /**
    * This function is called every 20 ms, no matter the mode. Use this for items
    * like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>
    * This runs after the mode specific periodic functions, but before LiveWindow
    * and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // Save shooter pivot positions
        double[] positions = ShooterSubsystem.getInstance().getPivotPositions();
        JSONManager.getInstance().saveShooterPivotPositions(positions[0], positions[1]);
    }

    @Override
    public void disabledPeriodic() {}

    /**
    * This autonomous runs the autonomous command selected by your
    * {@link RobotContainer} class.
    */
    @Override
    public void autonomousInit() {
        this.auton = RobotContainer.getInstance().getAutonomousCommand();
        if (this.auton != null) {
            try {
                this.auton.schedule();
            }
            catch (Exception error) {
                error.printStackTrace();
            }
        }
        else {
            System.err.println("No auton command found");
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (this.auton != null) {
            this.auton.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
