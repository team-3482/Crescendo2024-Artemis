// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {
  public enum LEDState {
    COLOR,
    RAINBOW
  };

  public static AddressableLED led = new AddressableLED(LEDConstants.LED_PORT);
  public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
  private int rainbowFirstPixelHue = 0;
  private int r, g, b;
  private LEDState state = LEDState.RAINBOW;

  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {
    led.setLength(ledBuffer.getLength());
    led.start();
  }

  public void SetColor(int r, int g, int b, LEDState state) {
    this.r = r;
    this.g = g;
    this.b = b;
    this.state = state;
  }

  private void Color() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
  }

  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
  private void Rainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 360 / ledBuffer.getLength())) % 360;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  @Override
  public void periodic() {
    switch (state) {
      case RAINBOW:
        Rainbow();
      case COLOR:
        Color();
    }

    led.setData(ledBuffer);
  }
}
