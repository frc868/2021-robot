package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * This is the RobotMap class, which stores constants for use throughout the
 * project. These constants include motor controller constants, controller
 * mappings, etc.
 * @author common
 */

public class RobotMap {
    public static class Camera {
        public static final double PIPELINE = 0.0;
    }

    public static class Climber {

    }

    public static class Drivetrain {
        public static final int LEFT_PRIMARY = 0; // TODO: untested
        public static final int LEFT_SECONDARY = 1; // TODO: untested
        public static final int RIGHT_PRIMARY = 2; // TODO: untested
        public static final int RIGHT_SECONDARY = 3; // TODO: untested

        public static final boolean LEFT_IS_INVERTED = false; // TODO: untested
        public static final boolean RIGHT_IS_INVERTED = false; // TODO: untested
    }

    public static class Hopper {
        public static final double ENC_COUNT_PER_CYCLE = 9127; // TODO: untested
        public static final double BELT_SPEED = 0.1; // TODO: untested
        public static final double INDEXER_SPEED = 0.1; // TODO: untested

        public static class Limit {
            public static final int BOTTOM_LEFT = 0; // TODO: untested
            public static final int BOTTOM_RIGHT = 1; // TODO: untested
            public static final int MIDDLE_LEFT = 2; // TODO: untested
            public static final int MIDDLE_RIGHT = 3; // TODO: untested
            public static final int TOP_LEFT = 4; // TODO: untested
            public static final int TOP_RIGHT = 5; // TODO: untested
        }

        public static class Motor {
            public static final int BELT = 2; // TODO: untested
            public static final int INDEXER = 21; // TODO: untested
            public static final boolean BELT_IS_INVERTED = true; // TODO: untested
            public static final boolean INDEXER_IS_INVERTED = true; // TODO: untested
        }
    }

    public static class Intake {
        public static final int PRIMARY = 0; // TODO: untested
        public static final int SECONDARY = 1; // TODO: untested
    }

    public static class LED {
        public static final int PORT = 4; // TODO: untested
        public static final int LENGTH = 27; // TODO: untested
    }

    public static class Shooter {
        public static final int MOTOR = 0; // TODO: untested
        public static final boolean MOTOR_IS_INVERTED = true; // TODO: untested
    }

    public static class Turret {
        public static final int MOTOR = 5; // TODO: untested
        public static final boolean MOTOR_IS_INVERTED = false; // TODO: untested
    }

    public static class WheelOfFortune {
        public static final int MOTOR = 21; // TODO: untested
        public static final int ACTUATOR = 0; // TODO: untested
        public static final boolean ACTUATOR_ENABLED_STATE = false; // TODO: untested

        public static final I2C.Port COLOR_SENSOR = I2C.Port.kMXP; // TODO: untested
        public static final double[] RED_VALUES = {0.487, 0.360, 0.152};
        public static final double[] GREEN_VALUES = {0.197, 0.561, 0.240};
        public static final double[] BLUE_VALUES = {0.143, 0.427, 0.429};
        public static final double[] YELLOW_VALUES = {0.361, 0.524, 0.113};
        public static final double[] BLANK = {0, 0, 0};

    }

    public static class Sensors {
        public static final Port GYRO = SPI.Port.kMXP;
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
