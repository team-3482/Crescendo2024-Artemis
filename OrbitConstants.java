/** Constants used for orbiting April Tags */
public static final class OrbitConstants {
    /** Multipies by the chasis speeds to slow down the bot for more control when orbitting */
    public static final double ORBIT_SPEED_COEFFIECENT = 0.25;
    /**
     * Multipies by the chasis speeds to slow down the bot for more control when
     * orbitting and using fine control
     */
    public static final double ORBIT_FINE_CONTROL_SPEED_COEFFIECENT = 0.125;

    /** The rate of change limit (units per second) for turning limiter in orbit mode */
    public static final double ORBIT_TURNING_SLEW_RATE_LIMIT = SwerveKinematics.TURNING_SLEW_RATE_LIMIT;
    /** The rate limit in units per second for driving in orbit mode (x and y) */
    public static final double ORBIT_DRIVE_SLEW_RATE_LIMIT = SwerveKinematics.DRIVE_SLEW_RATE_LIMIT;

    /** PID constants for controlling the turning speed during orbits */
    public static final class TURNING_SPEED_PID_CONTROLLER {
        /** Tolerance for the PID controller in degrees */
        public static final double TOLERANCE = 0.5;
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0;
    }

    /** Position in space to orbit (SPEAKER) */
    public static final Map<DriverStation.Alliance, Translation3d> ORBIT_POINT = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Red,
            new Translation3d(16.5, 5.55, 2.47)),
        Map.entry(DriverStation.Alliance.Blue,
            new Translation3d(0, 5.55, 2.47))
    );

    /** The AprilTags to center on at each SPEAKER */
    public static final Map<DriverStation.Alliance, Integer> ORBIT_TAG = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Red, 4),
        Map.entry(DriverStation.Alliance.Blue, 7)
    );
}