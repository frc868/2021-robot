package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_SparkMAX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;
import frc.robot.sensors.PotLimit;

/** 
 * This is the code for the shooter. It initializes motor controllers and has methods
 * for various functions of the shooter. It also uses PID control for maintaining optimal velocity.
 * Is probably unfinished and subject to massive revision
 * 
 * @author ama
 */

 public class Shooter {
    private static Shooter instance = null;

    private WPI_TalonSRX motor;
    private AnalogPotentiometer potentiometer;
    private PotLimit limit;

    private PIDController pid;
    private PIDSource pidSource;
    private PIDOutput pidOutput;

    // Setpoints and Potentiometer limits
    private final double LOWER = 0.85, MIDDLE = 0.831, UPPER = 0.726;
    private final boolean BRAKE_MODE = false;
    private static final double kP = 14, kI = 1, kD = 15;

    // Set to true if at upper limit
    private boolean limitPower = false;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        motor = new WPI_SparkMAX(RobotMap.Shooter.MOTOR);
        brake = new Solenoid(RobotMap.Shooter.BRAKE);
        limit = new PotLimit(potentiometer, UPPER, LOWER);

        motor.setInverted(true);

        pidSource = new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                // should not be used
            }
        
            @Override
            public double pidGet() {
                return getPotPosition();
            }
        
            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }
        };

        pidOutput = new PIDOutput() {
            @Override
            public void pidWrite(double output) {
                setSpeed(Helper.boundValue(-output, -0.3, 0.6));
                // These values may have to be changed
            }
        };

        pid = new PIDController(kP, kI, kD, pidSource, pidOutput);
        pid.setAbsoluteTolerance(0.001);
        // This value may have to be changed

        public void setSpeed(double speed) {
            setShooterBrake(false);
    
            if (limitPower) {
                motor.set(Helper.boundValue(speed, -0.25, 0.25));
                // These values may have to be changed
            } else {
                motor.set(Helper.boundValue(speed));
            }
    
            setBrake(speed == 0.0);
        }
        
        public void stop() {
            motor.stopMotor();
        }

        public double getPotPosition() {
            return potentiometer.get();
        }
        
        public void setPosition(ShooterPosition pos) {
            if (!pid.isEnabled()) {
                startPid();
            }
            stopBrake();
            this.limitPower = false;
            switch (pos) {
                case LOW:
                    pid.setSetpoint(LOWER);
                    break;
                case MIDDLE: 
                    pid.setSetpoint(MIDDLE);
                    break;
                case HIGH:
                    this.limitPower = true;
                    pid.setSetpoint(UPPER);
                    break;
                default:
                    break;
            }
        }
        
        /**
        * Starts the PID controller.
        */
        public void startPid() {
            pid.enable();
        }

        /**
        * Stops the PID controller.
        */
        public void stopPid() {
            pid.disable();
        }
    }
}
