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
    RAINBOW,
    RANDOM,
    GRADIENT
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

  private void Random() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i,
          (int) Math.random() * 255,
          (int) Math.random() * 255,
          (int) Math.random() * 255);
    }
  }

  private void Gradient(int color1[], int color2[]) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      float fraction = (float) i / ledBuffer.getLength();

      int red = (int) (color1[0] + fraction * (color2[0] - color1[0]));
      int green = (int) (color1[1] + fraction * (color2[1] - color1[1]));
      int blue = (int) (color1[2] + fraction * (color2[2] - color1[2]));

      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  @Override
  public void periodic() {
    switch (state) {
      case RAINBOW:
        Rainbow();
      case COLOR:
        Color();
      case RANDOM:
        Random();
      case GRADIENT:
        Gradient(LEDConstants.RED_COLOR, LEDConstants.BLUE_COLOR);
    }

    led.setData(ledBuffer);
  }
}
