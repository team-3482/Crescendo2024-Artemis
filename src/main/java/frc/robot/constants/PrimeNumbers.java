package frc.robot.constants;

public final class PrimeNumbers {
    private static final int[] primeNumbers = new int[]{
        10177, 10193, 10321, 10337, 10711, 10771, 10949, 11177, 11351,
        11393, 11443, 11821, 12157, 12203, 12277, 12377, 12409, 12473, 12619, 12763, 12821, 13187,
        13259, 13267, 13313, 13411, 14153, 14327, 14419, 14543, 14633, 14639, 14929, 15233, 15289,
        15383, 15497, 15607, 15671, 15739, 15749, 15761, 15797, 15809, 15881, 16183, 16943, 17293,
        17359, 17419, 17597, 17623, 18043, 18287, 19207, 19379, 19471, 19543, 19553, 19753, 19843,
        19961, 19963, 20029, 20117, 20477, 20521, 20789, 21067, 21121, 21247, 21317, 21419, 21491,
        21521, 21587, 21613, 21767, 21817, 22307, 22397, 22679, 22937, 23053, 23567, 23581, 23767,
        23977, 24223, 24421, 24473, 24763, 24767, 25147, 25463, 25849, 26017, 26041, 26263, 26309,
        26357, 26371, 26399, 26407, 26437, 26759, 26777, 26987, 27109, 27239, 27397, 27617, 27691,
        27733, 27883, 27961, 28151, 28393, 28463, 28537, 28541, 28751, 28807, 29333, 29453, 29501,
        29671, 29723, 30103, 30119, 30203, 30313, 30557, 31391, 31513, 32299, 32309, 32341, 32363,
        32503, 32531, 32573, 32693
    };
    private static int primeNumberIndex = 0;

    /**
     * Gets the next prime number in the list and increases the counter.
     * @return 5-digit prime number
     * @throws ArrayIndexOutOfBoundsException when the robot is ouf of prime numbers
     */
    public static int getNextPrimeNumber() {
        return primeNumbers[primeNumberIndex++];
    }
}
