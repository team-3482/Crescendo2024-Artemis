// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.awt.Color;

public class LEDSubsystem extends SubsystemBase {
  public enum LEDState {
    COLOR,
    RAINBOW,
    GRADIENT,
    FADE
  };

  public static AddressableLED led = new AddressableLED(LEDConstants.ledPort);
  public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.ledCount);
  private int rainbowFirstPixelHue = 0;
  private boolean gradientCountUp = true;
  private int gradientIndex;
  private int fadeIndex;
  private int rgb[] = { 0, 0, 0 };
  private LEDState state = LEDState.RAINBOW;

  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {
    led.setLength(ledBuffer.getLength());
    led.start();
  }

  public void SetColor(int[] rgb, LEDState state) {
    this.rgb[0] = rgb[0];
    this.rgb[1] = rgb[1];
    this.rgb[2] = rgb[2];
    this.state = state;
  }

  private void Color() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
    }
  }

  private void Rainbow() { // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 360 / ledBuffer.getLength())) % 360;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  private void Gradient(int color1[], int color2[]) {
    // color1 hue NEEDS to be less than color2.
    // i am too lazy to make a better way to do this.
    int color1_hue = Math.round(Color.RGBtoHSB(color1[0], color1[1], color1[2], null)[0]);
    int color2_hue = Math.round(Color.RGBtoHSB(color2[0], color2[1], color2[2], null)[0]);

    gradientIndex = color1_hue;

    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, gradientIndex, 255, 255);
      if (gradientIndex >= color2_hue) {
        gradientCountUp = false;
      } else if (gradientIndex <= color1_hue) {
        gradientCountUp = true;
      }

      if (gradientCountUp) {
        gradientIndex++;
      } else {
        gradientIndex--;
      }
    }
  }

  // fade is just gradient but it doesnt reset the color back
  private void Fade(int color1[], int color2[]) {
    int color1_hue = Math.round(Color.RGBtoHSB(color1[0], color1[1], color1[2], null)[0]);
    int color2_hue = Math.round(Color.RGBtoHSB(color2[0], color2[1], color2[2], null)[0]);

    fadeIndex = color1_hue;

    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, gradientIndex, 255, 255);

      if (gradientIndex >= color2_hue) {
        gradientIndex = color1_hue;
      }

      gradientIndex++;
    }
  }

  @Override
  public void periodic() {
    switch (state) {
      case RAINBOW:
        Rainbow();
      case COLOR:
        Color();
      case GRADIENT:
        Gradient(LEDConstants.redColor, LEDConstants.blueColor);
      case FADE:
        Fade(LEDConstants.redColor, LEDConstants.blueColor);
    }

    led.setData(ledBuffer);
  }
}
