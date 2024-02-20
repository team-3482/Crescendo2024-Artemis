package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip {
    
    // LED
    private AddressableLED led;
    // LED Buffer
    private AddressableLEDBuffer ledBuffer;

    public LEDStrip(int portNumber, int length)
    {
        this.ledBuffer = new AddressableLEDBuffer(length);

        this.led = new AddressableLED(portNumber);
        this.led.setLength(length);
        this.led.start();
    }

    public void setColor(Color color)
    {
        // Assign Colors
        for (int i = 0; i < this.ledBuffer.getLength(); i++) {
            this.ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }

        this.led.setData(this.ledBuffer);
    }

}
