// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.LEDConstants;
import frc.robot.shooter.SterilizerSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

    // LED Buffer
    private AddressableLEDBuffer ledBuffer;
    // LED 
    private AddressableLED ledStrip;

    private double lastLedUpdate = 0.0;
    private LightState state;
    private Color previousColor;

    private SimpleWidget SB_D_LED_WIDGET = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .add("LED Status", false);
    private GenericEntry SB_D_LED_ENTRY = SB_D_LED_WIDGET
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withPosition(0, 3)
        .withSize(3, 2)
        .getEntry();
    
    /**
     * Creates and initializes a new LEDSubsystem
     */
    private LEDSubsystem() {
        super("LEDSubsystem");
        
        this.ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

        this.ledStrip = new AddressableLED(LEDConstants.LED_PORT);
        this.ledStrip.setLength(LEDConstants.LED_COUNT);
        this.ledStrip.start();

        this.lastLedUpdate = Timer.getFPGATimestamp();

        this.state = LightState.OFF;
        this.previousColor = this.state.getColor();

    }

    @Override
    public void periodic() { 
        // Updates the color effect and gets the chosen color
        if (this.state.interval != Double.POSITIVE_INFINITY) {
            double timestamp = Timer.getFPGATimestamp();
            if (timestamp - this.lastLedUpdate >= this.state.interval) {
                this.state.cycleColors();
                this.lastLedUpdate = timestamp;
            }
        }

        // Updates the lights only if the light state changes
        if (!this.state.getColor().equals(this.previousColor)) {
            updateLights();
        }
    };

    /**
     * Updates the light colors of the light strips
     */
    private void updateLights() {
        Color color = this.state.getColor();
        
        if (color.equals(Color.off())) {
            SB_D_LED_ENTRY.setBoolean(false);
        }
        else {
            SB_D_LED_WIDGET.withProperties(Map.of("colorWhenTrue", color.getHexadecimal()));
            SB_D_LED_ENTRY.setBoolean(true);
        }
        
        for (int i = 0; i < this.ledBuffer.getLength(); i++) {
            this.ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }

        this.ledStrip.setData(this.ledBuffer);

        this.previousColor = this.state.getColor();
    }

    /**
     * Sets the state of the lights on the bot.
     * @param state Desired {@link LightState}
     */
    public void setLightState(LightState state) {
        System.out.println("LED State changed:" + state);
        this.setLightState(state, true);
    }

    /**
     * Sets the state of the lights on the bot. 
     * If ovverideCurrent is true, it will override current colors, otherwise it will only change colors if the current state is off
     * @param state Desired {@link LightState}
     * @param overrideCurrentState Whether the lights should overide the current lightt state or not.
     */
    public void setLightState(LightState state, boolean overrideCurrentState){
        if (overrideCurrentState || (!overrideCurrentState && this.state.equals(LightState.OFF))) {
            this.state = state;
        }
    }

    /**
     * Reset the lights to the default {@link LightState}.
     * @param commandInterrupted set warning lights instead if true
     */
    public void setCommandStopState(boolean commandInterrupted) {
        if (commandInterrupted) {
            this.setLightState(LightState.WARNING);
        }
        else {
            this.setLightState(SterilizerSubsystem.getInstance().hasNote() == true ? LightState.HOLDING_NOTE : LightState.OFF);
        }
    }
    
    public enum LightState { 
        OFF (Double.POSITIVE_INFINITY, Color.off()),
        WARNING (0.2, new Color(255, 0, 0), Color.off()),
        
        /** Command is considered not autonoumous if the human driver has control over the robot movement during the command */
        CMD_RUNNING (Double.POSITIVE_INFINITY, new Color(0, 255, 0)),
        /** Command is considered autonoumous if the human driver does not have control over the robot movement during the command */
        AUTO_RUNNING (Double.POSITIVE_INFINITY, new Color(0, 0, 255)),
        
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
