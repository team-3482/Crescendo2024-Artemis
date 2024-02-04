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
  private int gradientIndex, fadeIndex, r, g, b;
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

    for (int i = 0; i <= ledBuffer.getLength(); i++) {
      int[] transitionColor = new int[3];

      for (int j = 0; j < 3; i++) {
        transitionColor[i] = color1[i] + (color2[i] - color1[i]) * i / ledBuffer.getLength();
      }
      for (int step = 0; step <= steps; step++) {
        int[] transitionColor = new int[3];

        // Interpolate values for each color channel
        for (int i = 0; i < 3; i++) {
          transitionColor[i] = color1[i] + (color2[i] - color1[i]) * step / steps;
        }

        // Use the transitionColor array for further processing or display
        // For example, you can print the values
        System.out.println(
            "Step " + step + ": [" + transitionColor[0] + ", " + transitionColor[1] + ", " + transitionColor[2] + "]");
      }

      // Use the transitionColor array for further processing or display
      // For example, you can print the values
      System.out.println(
          "Step " + i + ": [" + transitionColor[0] + ", " + transitionColor[1] + ", " + transitionColor[2] + "]");
    }

    // for (var i = 0; i < ledBuffer.getLength(); i++) {
    // ledBuffer.setHSV(i, gradientIndex, 100, 100);
    // if (gradientIndex >= color2_hue) {
    // gradientCountUp = false;
    // } else if (gradientIndex <= color1_hue) {
    // gradientCountUp = true;
    // }
    //
    // if (gradientCountUp) {
    // gradientIndex++;
    // } else {
    // gradientIndex--;
    // }
    // }
  }

  // fade is just gradient but it doesnt reset the color back
  private void Fade(int color1[], int color2[]) {
    int color1_hue = Math.round(Color.RGBtoHSB(color1[0], color1[1], color1[2], null)[0]);
    int color2_hue = Math.round(Color.RGBtoHSB(color2[0], color2[1], color2[2], null)[0]);

    fadeIndex = color1_hue;

    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, gradientIndex, 255, 255);

      if (fadeIndex >= color2_hue) {
        fadeIndex = color1_hue;
      }

      fadeIndex++;
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
