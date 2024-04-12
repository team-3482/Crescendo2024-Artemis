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

/**
 * A subsystem used to change the colors of the LED strip below the robot.
 */
public class LEDSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile LEDSubsystem instance;
    private static Object mutex = new Object();

    public static LEDSubsystem getInstance() {
        LEDSubsystem result = instance;

        if(result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null)
                    instance = result = new LEDSubsystem();
            }
        }
        return instance;
    }

    private AddressableLEDBuffer ledBuffer;
    private AddressableLED ledStrip;

    private double lastLedUpdate;
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
     * Creates a new LEDSubsystem.
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
        // Updates the color effect and gets the chosen color.
        if (this.state.interval != Double.POSITIVE_INFINITY) {
            double timestamp = Timer.getFPGATimestamp();
            if (timestamp - this.lastLedUpdate >= this.state.interval) {
                this.state.cycleColors();
                this.lastLedUpdate = timestamp;
            }
        }

        // Updates the lights only if the light state should change.
        if (!this.state.getColor().equals(this.previousColor)) {
            updateLights();
        }
    };

    /**
     * Loops through the LEDs on the light strip and updates their color.
     * Also updates the Shuffleboard widget for the LED status.
     */
    private void updateLights() {
        Color color = this.state.getColor();
        
        if (color.equals(Color.off())) {
            SB_D_LED_ENTRY.setBoolean(false);
        }
        else {
            // The only way to display a color in Shuffleboard is by changing the color of a boolean widget.
            SB_D_LED_WIDGET.withProperties(Map.of("colorWhenTrue", color.getHexadecimal()));
            SB_D_LED_ENTRY.setBoolean(true);
        }
        
        for (int i = 0; i < this.ledBuffer.getLength(); i++) {
            this.ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }

        this.ledStrip.setData(this.ledBuffer);
        this.previousColor = color;
    }

    /**
     * Sets the state of the lights on the bot. 
     * @param state desired state.
     * @param overrideCurrentState whether the current light state should be overriden.
     */
    public void setLightState(LightState state, boolean overrideCurrentState) {
        if (overrideCurrentState || (!overrideCurrentState && this.state.equals(LightState.OFF))) {
            System.out.println("LED State changed: " + state);
            this.state = state;
        }
    }

    /**
     * Sets the state of the lights on the bot and overrides it.
     * @param state desired state.
     */
    public void setLightState(LightState state) {
        setLightState(state, true);
    }

    /**
     * Reset the lights to {@link LightState#OFF} or {@link LightState#WARNING}.
     * @param warning set warning lights
     */
    public void setCommandStopState(boolean warning) {
        if (warning) {
            setLightState(LightState.WARNING);
        }
        else {
            setLightState(SterilizerSubsystem.getInstance().hasNote() == true ? LightState.HOLDING_NOTE : LightState.OFF);
        }
    }
    
    public enum LightState { 
        OFF(Double.POSITIVE_INFINITY, Color.off()),
        WARNING(0.2, new Color(255, 0, 0), Color.off()),
        
        /** Command is considered not autonoumous if the driver has control over the robot movement during the command. */
        CMD_RUNNING(Double.POSITIVE_INFINITY, new Color(0, 255, 0)),
        /** Command is considered autonoumous if the driver does not have control over the robot movement during the command. */
        AUTO_RUNNING(Double.POSITIVE_INFINITY, new Color(0, 0, 255)),
        
        HOLDING_NOTE(Double.POSITIVE_INFINITY, new Color(255, 127, 0))
        ;
        
        Color[] colors;
        double interval;
        int currentColorIndex;
        
        /**
         * @param interval in seconds.
         * @param colors to display.
         */
        private LightState(double interval, Color... colors) {
            this.colors = colors;
            this.interval = interval;
            this.currentColorIndex = 0;
        }
        
        /**
         * Gets the current color.
         * @return current color.
         */
        public Color getColor() {
            return this.colors[this.currentColorIndex];
        }
        
        /**
         * Updates the current color index to get the next color in the color array.
         */
        public void cycleColors() {
            if (this.currentColorIndex >= this.colors.length - 1) {
                this.currentColorIndex = 0; 
            }
            else {
                this.currentColorIndex++;
            }
        }
    }
}
