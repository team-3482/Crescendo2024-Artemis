package frc.robot.lights;

/** 
 * Custom implementation of an object that holds information about colors.
 * This class is redundant and {@link edu.wpi.first.wpilibj.util.Color} should be used in the future.
 */
public class Color {
    private int red;
    private int green;
    private int blue;

    /**
     * Creates a new Color object.
     * @param r - red (0 - 255)
     * @param g - green (0 - 255)
     * @param b - blue (0 - 255)
     */
    public Color(int r, int g, int b) {
        this.red = r;
        this.green = g;
        this.blue = b;
    }
    
    /**
     * Creates and returns a Color object with no color values.
     * @return black.
     */
    public static Color off() {
        return new Color(0, 0, 0);
    }

    /**
     * Gets the red value.
     * @return red.
     */
    public int getRed() {
        return this.red;
    }

    /**
     * Gets the green value.
     * @return green.
     */
    public int getGreen() {
        return this.green;
    }

    /**
     * Gets the blue value.
     * @return blue.
     */
    public int getBlue() {
        return this.blue;
    }

    /**
     * Gets the hexadecimal value.
     * @return hexadecimal value.
     */
    public String getHexadecimal() {
        return String.format("#%02x%02x%02x", red, green, blue);  
    }

    public boolean equals(Color other) {
        return red == other.red && blue == other.blue && green == other.green;
    }
}
