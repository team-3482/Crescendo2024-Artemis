// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LEDSubsystem extends SubsystemBase {

  // Singleton Design Pattern
  private static LEDSubsystem instance;
  public static LEDSubsystem getInstance()
  {
    if(instance == null)
    {
      new LEDSubsystem();
    }
    return instance;
  }

  // LED
  private static AddressableLED led;
  // LED Buffer
  private static AddressableLEDBuffer ledBuffer;

  private double lastLedUpdate = 0.0;
  private LightState state;
  
  /**
   * Creates and initializes a new LEDSubsystem
   */
  public LEDSubsystem()
  {
    instance = this;

    ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

    led = new AddressableLED(LEDConstants.LED_PORT);
    led.setLength(ledBuffer.getLength());
    led.start();

    this.lastLedUpdate = Timer.getFPGATimestamp();
    this.state = LightState.OFF;
  }

  @Override
  public void periodic() { 
    // Updates the color effect and gets the chosen color
    if(this.state.interval != Double.POSITIVE_INFINITY)
    {
      double timestamp = Timer.getFPGATimestamp();
      if(timestamp - this.lastLedUpdate >= this.state.interval)
      {
        this.state.cycleColors();
        this.lastLedUpdate = timestamp;
      }
    }
    Color color = this.state.getColor();
    
    // Assign Colors
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }
    led.setData(ledBuffer);
  };

  /**
   * Sets the state of the lights on the bot
   * @param state - Desired LightState
   */
  public void setLightState(LightState state)
  {
    this.state = state;
  }

  
  public enum LightState { 
    OFF (Double.POSITIVE_INFINITY, Color.off()),
    WARNING (0.2, new Color(255, 0, 0)),
    
    SOLID_GREEN (Double.POSITIVE_INFINITY, new Color(0, 255, 0)),
    SOLID_BLUE (Double.POSITIVE_INFINITY, new Color(0, 0, 255)),
    SOLID_RED (Double.POSITIVE_INFINITY, new Color(255, 0, 0)),
    
    FLASHING_GREEN (0.2, new Color(0,255, 0), Color.off()),
    FLASHING_BLUE (0.2, new Color(0, 0, 255), Color.off()),
    ;
    
    Color[] colors;
    double interval;
    int currentColorIndex;
    int currentPixelOffset;
    /**
     * @param interval
     * @param blendMode
     * @param colors
     */
    private LightState(double interval, Color... colors)
    {
      this.colors = colors;
      this.interval = interval;
      this.currentColorIndex = 0;
      this.currentPixelOffset = 0;
    }
    /**
     * Gets the current color from array
     * @return current color
     */
    public Color getColor()
    {
      return this.colors[this.currentColorIndex];
    }
    /**
     * Updates the current color index to get the next color in the color array
     */
    public void cycleColors()
    {
      if(this.currentColorIndex >= this.colors.length - 1)
      {
        this.currentColorIndex = -1; // set to -1 to ensure doesnt skip first color
      }
      this.currentColorIndex++;
     }
  }

  private static class Color{
  
    private int red;
    private int green;
    private int blue;
  
    /**
     * Creates and initializes a new Color object 
     * @param r - red (0 - 255)
     * @param g - green (0 - 255)
     * @param b - blue (0 - 255)
     */
    public Color(int r, int g, int b)
    {
      this.red = r;
      this.green = g;
      this.blue = b;
    }
    /**
     * Creates and returns a Color object with no color values
     * @return Color with 0 color values
     */
    public static Color off()
    {
      return new Color(0,0,0);
    }
    /**
     * Gets the red value of the color
     * @return red value
     */
    public int getRed()
    {
      return this.red;
    }
    /**
     * Gets the green value of the color
     * @return green value
     */
    public int getGreen(){
      return this.green;
    }
    /**
     * Gets the blue value of the color
     * @return blue value
     */
    public int getBlue(){
      return this.blue;
    }
  }
}
