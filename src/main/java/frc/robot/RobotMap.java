package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * This is the RobotMap class, which stores constants for use throughout the
 * project. These constants include motor controller constants, controller
 * mappings, etc.
 * @author
 */

public class RobotMap {
    public static class Climber {

    }

    public static class Drivetrain {

    }

    public static class Intake {

    }

    public static class Shooter {
        public static final int MOTOR = 0; // TODO: untested
    }

    public static class WheelOfFortune {
        public static final int MOTOR = 21; // TODO: untested
        public static final I2C.Port COLOR_SENSOR = I2C.Port.kMXP; // TODO: untested
        public static final double[] RED_VALUES = {0.36,0.415,0.216};
        public static final double[] GREEN_VALUES = {0.22,0.501,0.275};
        public static final double[] BLUE_VALUES = {0.191,0.451,0.349};
        public static final double[] YELLOW_VALUES = {0.299,0.536,0.164};
        public static final double[] BLANK = {0,0,0};

    }

    public static class Sensors {
        public static final Port GYRO = SPI.Port.kMXP; //TODO: untested
    }

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RB = 6;
        public static final int LB = 5;
        public static final int RSTK = 10;
        public static final int LSTK = 9;
        public static final int START = 8;
        public static final int MENU = 7;

        public static final int LX = 0;
        public static final int LY = 1; // Arcade drive
        public static final int RX = 4; // Arcade drive

        public static final int RY = 5;
        public static final int LT = 2;
        public static final int RT = 3;

        public static final int POV = 0; // untested
    }
}
