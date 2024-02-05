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
  private AddressableLED led = new AddressableLED(LEDConstants.LED_PORT);
  // LED Buffer
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

  private double lastLedUpdate = 0.0;
  private LightState state;
  
  /**
   * Creates and initializes a new LEDSubsystem
   */
  public LEDSubsystem()
  {
    instance = this;

    this.led = new AddressableLED(LEDConstants.LED_PORT);
    this.led.setLength(ledBuffer.getLength());
    this.led.start();

    this.ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

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
      this.ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }
    this.led.setData(this.ledBuffer);
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
    
    RED_BLUE_GRADIENT (Double.POSITIVE_INFINITY, LightBlendMode.GRADIENT, new Color(255, 0, 0), new Color(0, 0, 255)),
    RED_BLUE_SCROLL (Double.POSITIVE_INFINITY, LightBlendMode.SCROLL, new Color(255, 0, 0), new Color(0, 0, 255));

    Color[] colors;
    double interval;
    int currentColorIndex;
    LightBlendMode blendMode;
    /**
     * @param interval
     * @param blendMode
     * @param colors
     */
    private LightState(double interval, LightBlendMode blendMode, Color... colors)
    {
      this.blendMode = blendMode;
      switch (this.blendMode){
        case SCROLL:
        case GRADIENT:
          this.colors = Color.createGradient(150, colors);
          this.interval = Double.POSITIVE_INFINITY;
          this.currentColorIndex = 0;
          break;
        default:
          this.colors = colors;
          this.interval = interval;
          this.currentColorIndex = 0;
          break;
      }
    }
    private LightState(double interval, Color... colors)
    {
      this(interval, LightBlendMode.SOLID, colors);
    }
    /**
     * Gets the current color from array
     * @return current color
     */
    public Color getColor()
    {
      if(this.blendMode == LightBlendMode.SOLID)
      {
        return this.colors[this.currentColorIndex];
      }
      else
      {
        cycleColors();
        return this.colors[this.currentColorIndex];
      }
    }

    /**
     * Updates the current color index to get the next color in the color array
     */
    public void cycleColors()
    {
      if(this.currentColorIndex >= this.colors.length)
      {
        this.currentColorIndex = -1; // set to -1 to ensure doesnt skip first color
      }
      this.currentColorIndex++;
     }
  }
  private enum LightBlendMode{
    SOLID,
    GRADIENT, SCROLL
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
    /**Linear interpolation between two colors 
     * (http://www.java2s.com/example/java-utility-method/color-interpolate/interpolate-color-low-color-high-double-min-double-max-double-v-64b61.html)
     * @param low Color to associate with the min value
     * @param high Color to associate with the max value
     * @param min Smallest value that will be passed
     * @param max Largest value that will be passed
     * @param v Current value 
     * **/
    public static Color interpolate(Color low, Color high, double min, double max, double v) {
      if (v > max) {
          v = max;
      }
      if (v < min) {
          v = min;
      }
      double distance = 1 - ((max - v) / (max - min));
      if (Double.isNaN(distance) || Double.isInfinite(distance)) {
          return high;
      }
      int r = (int) weightedAverage(high.getRed(), low.getRed(), distance);
      int g = (int) weightedAverage(high.getGreen(), low.getGreen(), distance);
      int b = (int) weightedAverage(high.getBlue(), low.getBlue(), distance);
      return new Color(r, g, b);
  }
    /**Weighted average between two values
     * 
     * @param min The lowest value to expect
     * @param max The highest value to expect
     * @param p The desired percentage offset between max and min 
     * @return The resulting value
     */
    public static double weightedAverage(double min, double max, double p) {
      return (min - max) * p + max;
    }

    public static Color[] createGradient(int length, Color[] colors)
    {
      Color[] gradient = new Color[length];
      int distanceBetweenColors = length / colors.length;
      int indexOffset = 0;
      for(int i = 1; i < colors.length; i++)
      {
        int lowIndex = indexOffset;
        int highIndex = indexOffset + distanceBetweenColors;
        for(int j=0; j < distanceBetweenColors; j++)
        {
          gradient[indexOffset] = interpolate(colors[i-1], colors[i], lowIndex, highIndex, indexOffset);
          indexOffset++;
        }
      }

      return gradient;
    }
  }
}
