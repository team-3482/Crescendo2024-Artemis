// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShuffleboardTabConstants;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class LEDSubsystem extends SubsystemBase {
    // Singleton Design Pattern
    private static LEDSubsystem instance;
    public static LEDSubsystem getInstance() {
        if(instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }
    
    private LEDStrip underGlowStrip;
    // private LEDStrip leftElevatorStrip;
    // private LEDStrip rightElevatorStrip;

    private double lastLedUpdate = 0.0;
    private LightState state;
    private LightState defaultState;

    private SimpleWidget SB_D_LED_WIDGET = Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
        .add("LED Status", false);
    private GenericEntry SB_D_LED_ENTRY = SB_D_LED_WIDGET
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withPosition(9, 0)
        .withSize(6, 3)
        .getEntry();
    
    /**
     * Creates and initializes a new LEDSubsystem
     */
    private LEDSubsystem() {
        super("LEDSubsystem");
        this.underGlowStrip = new LEDStrip(LEDConstants.UNDERGLOW_LED_PORT, LEDConstants.UNDERGLOW_LED_COUNT);
        // this.leftElevatorStrip = new LEDStrip(LEDConstants.LEFT_ELEVATOR_LED_PORT, LEDConstants.ELEVATOR_LED_COUNT);
        // this.rightElevatorStrip = new LEDStrip(LEDConstants.RIGHT_ELEVATOR_LED_PORT, LEDConstants.ELEVATOR_LED_COUNT);
        this.lastLedUpdate = Timer.getFPGATimestamp();

        this.defaultState = LightState.OFF;
        this.state = this.defaultState;
    }

    @Override
    public void periodic() { 
        // Updates the color effect and gets the chosen color
        if(this.state.interval != Double.POSITIVE_INFINITY) {
            double timestamp = Timer.getFPGATimestamp();
            if (timestamp - this.lastLedUpdate >= this.state.interval) {
                this.state.cycleColors();
                this.lastLedUpdate = timestamp;
            }
        }
        Color color = this.state.getColor();
      
        if (color.equals(Color.off())) {
            SB_D_LED_ENTRY.setBoolean(false);
        }
        else {
            SB_D_LED_WIDGET.withProperties(Map.of("colorWhenTrue", color.getHexadecimal()));
            SB_D_LED_ENTRY.setBoolean(true);
        }

        underGlowStrip.setColor(color);
        // leftElevatorStrip.setColor(color);
        // rightElevatorStrip.setColor(color);

    };

    /**
     * Sets the state of the lights on the bot.
     * 
     * @param state Desired {@link LightState}
     */
    public void setLightState(LightState state) {
        this.state = state;
    }

    /**
     * Sets the default state of the lights on the bot
     * 
     * @param state Desired {@link LightState}
     */
    public void setDefaultLightState(LightState state) {
        this.defaultState = state;
        this.setCommandStopState(false);
    }

    /**
     * Reset the lights to the default {@link LightState}.
     * 
     * @param commandInterrupted set warning lights instead if true
     */
    public void setCommandStopState(boolean commandInterrupted) {
        if (commandInterrupted) {
            this.setLightState(LightState.WARNING);
        }
        else {
            this.setLightState(this.defaultState);
        }
    }
    
    public enum LightState { 
        OFF (Double.POSITIVE_INFINITY, Color.off()),
        WARNING (0.2, new Color(255, 0, 0), Color.off()),
        
        /** Command is considered not autonoumous if the human driver has control over the robot movement during the command */
        CMD_RUNNING (Double.POSITIVE_INFINITY, new Color(0, 255, 0)),
        /** Command is considered autonoumous if the human driver does not have control over the robot movement during the command */
        AUTO_RUNNING (Double.POSITIVE_INFINITY, new Color(0, 0, 255)),
        /** For when the command is initializing (if you see this color, there is an issue) */
        CMD_INIT(Double.POSITIVE_INFINITY, new Color(255, 255, 0)),
        
        HOLDING_NOTE(Double.POSITIVE_INFINITY, new Color(255, 127, 0))
        ;
        
        Color[] colors;
        double interval;
        int currentColorIndex;        
        /**
         * @param interval
         * @param colors
         */
        private LightState(double interval, Color... colors) {
            this.colors = colors;
            this.interval = interval;
            this.currentColorIndex = 0;
        }
        /**
         * Gets the current color from array
         * @return current color
         */
        public Color getColor() {
            return this.colors[this.currentColorIndex];
        }
        /**
         * Updates the current color index to get the next color in the color array
         */
        public void cycleColors() {
            if(this.currentColorIndex >= this.colors.length - 1) {
                this.currentColorIndex = -1; // set to -1 to ensure doesnt skip first color
            }
            this.currentColorIndex++;
        }
    }
}
