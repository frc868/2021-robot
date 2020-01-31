package frc.robot;

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
        public static final double SHOOTER_PIPELINE = 1.0;
        public static final double BALL_PIPELINE = 2.0;
    }

    public static class Climber {

    }

    public static class Drivetrain {

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

    public static class Shooter {
        public static final int MOTOR = 0; // TODO: untested
        public static final boolean MOTOR_IS_INVERTED = true; // TODO: untested
    }

    public static class Turret {
        public static final int MOTOR = 15; //TODO: untested
        public static final boolean MOTOR_IS_INVERTED = false; // TODO: untested
    }

    public static class WheelOfFortune {

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
