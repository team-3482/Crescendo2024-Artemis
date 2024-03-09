package frc.robot.lights;

public class Color {
    private int red;
    private int green;
    private int blue;

    /**
     * Creates and initializes a new Color object 
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
     * Creates and returns a Color object with no color values
     * @return Color with 0 color values
     */
    public static Color off() {
        return new Color(0,0,0);
    }
    /**
     * Gets the red value of the color
     * @return red value
     */
    public int getRed() {
        return this.red;
    }
    /**
     * Gets the green value of the color
     * @return green value
     */
    public int getGreen() {
        return this.green;
    }
    /**
     * Gets the blue value of the color
     * @return blue value
     */
    public int getBlue() {
        return this.blue;
    }
    /**
     * Gets the hexadecimal value of che color
     * @return hexadecimal value
     */
    public String getHexadecimal() {
        return String.format("#%02x%02x%02x", red, green, blue);  
    }
    public boolean equals(Color other) {
        return red == other.red && blue == other.blue && green == other.green;
    }
}
