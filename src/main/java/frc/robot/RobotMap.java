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
        public static final int LEFT_PRIMARY = 12;
        public static final int LEFT_SECONDARY = 13;
        public static final int RIGHT_PRIMARY = 14;
        public static final int RIGHT_SECONDARY = 15;

        public static final boolean LEFT_IS_INVERTED = false;
        public static final boolean RIGHT_IS_INVERTED = true;
    }

    public static class Hopper {
        public static final double ENC_COUNT_PER_CYCLE = 9127; // TODO: untested
        public static final double HOPPER_FLOOR_SPEED = 1; // TODO: untested
        public static final double FEEDER_SPEED = 1; // TODO: untested
        public static final double BLUE_SPEED = 1;

        public static class Sensors {
            public static final int BOT_SENSOR_PORT = 5; // TODO: untested
            public static final int MID_LEFT = 3; // TODO: untested
            public static final int TOP_LEFT = 2; // TODO: untested

            public static final boolean TOP_LAST_STATE_VALUE = false; // TODO: untested
            public static final boolean MID_LAST_STATE_VALUE = false; // TODO: untested
            public static final boolean BOT_LAST_STATE_VALUE = true; // TODO: untested
        }

        public static class Motor {
            public static final int HOPPER_FLOOR = 10; // TODO: untested
            public static final int BLUE_WHEELS = 11; // TODO: untested
            public static final int FEEDER = 3; // TODO: untested
            public static final boolean HOPPER_FLOOR_IS_INVERTED = true; // TODO: untested
            public static final boolean FEEDER_IS_INVERTED = false; // TODO: untested
        }
    }

    public static class Intake {
        public static final int MOTOR = 1; // TODO: untested
        public static final boolean MOTOR_IS_INVERTED = true; // TODO: untested
        public static final int ACTUATOR1 = 0; // TODO: untested
        public static final int ACTUATOR2 = 7; // TODO: untested
    }

    public static class LED {
        public static final int PORT = 4; // TODO: untested
        public static final int LENGTH = 27; // TODO: untested
    }

    public static class Shooter {
        public static final int PRIMARY = 7; // TODO: untested
        public static final int SECONDARY = 8; // TODO: untested
        public static final boolean PRIMARY_IS_INVERTED = true; // TODO: untested
        public static final boolean SECONDARY_IS_INVERTED = true; // TODO: untested
    }

    public static class Turret {
        public static final int MOTOR = 2;

        public static class CompBot {
            public static final boolean MOTOR_IS_INVERTED = false;
            public static final int ENCODER_1 = 0;
            public static final int ENCODER_2 = 1;

            public static class Limits {
                public static final int LEFT_PORT = 6; // TODO: untested
                public static final int RIGHT_PORT = 7; // TODO: untested

                public static final boolean LIMIT_TRIGGERED = false;
            }

            public static class PID {
                public static final double kP = 0.012;
                public static final double kI = 0;
                public static final double kD = 0.0001;
            }
        }

        public static class PracticeBot {
            public static final boolean MOTOR_IS_INVERTED = false; // TODO: untested

            public static class Limits {
                public static final int LEFT_PORT = 0; // TODO: untested
                public static final int RIGHT_PORT = 1; // TODO: untested

                public static final boolean LIMIT_TRIGGERED = true;
            }

            public static class PID {
                public static final double kP = 0.015; // TODO: untested
                public static final double kI = 0; // TODO: untested
                public static final double kD = 0.0001; // TODO: untested
            }
        }


        

        public static class Setpoints {
            public static final double SAFE_POSITION = 0; // TODO: untested
            public static final double DEADZONE_LEFT = 0; // TODO: untested
            public static final double DEADZONE_RIGHT = 23000; // TODO: untested
        }
    }

    public static class WheelOfFortune {
        public static final int MOTOR = 4; // TODO: untested
        public static final int ACTUATOR1 = 2;
        public static final int ACTUATOR2 = 5;
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
