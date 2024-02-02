// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LED;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {
  public static AddressableLED led = new AddressableLED(0);
  public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(150);

  private int rainbowFirstPixelHue = 0;
  private int r = 0;
  private int g = 0;
  private int b = 0;
  private boolean isRainbow = true;

  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {
    led.setLength(ledBuffer.getLength());
    led.start();
  }

  public void SetColor(int r, int g, int b, boolean isRainbow) {
    this.r = r;
    this.g = g;
    this.b = b;
    this.isRainbow = isRainbow;
  }

  private void rainbow() { // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 360 / ledBuffer.getLength())) % 360;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
  }

  @Override
  public void periodic() {
    if (isRainbow) {
      rainbow();
    } else {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, r, g, b);
      }
    }

    led.setData(ledBuffer);
  }
}
