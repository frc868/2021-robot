package frc.robot.subsystems;

/** 
 * This is the code for the robot drivetrain. It initializes motor controllers and has methods
 * for various functions of the drivetrain.
 * @author 
 */
public class Drivetrain {
    private static Drivetrain instance;

    private Drivetrain() {

    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            return new Drivetrain();
        }
        return instance;
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        
    }

    public void driveStraight(double targetDist, double startPower, double endPower) {
        double pGain = 0.5;
        double initialDist = getAveragePosition();
        double distanceToTarget = Math.abs(targetDist) - Math.abs(getAveragePosition() - initialDist);

        double targetSpeed = pGain * (startPower + ((endPower - startPower) / distanceToTarget));

        if (distanceToTarget > 0) {
            setSpeed(targetSpeed, targetSpeed); // TODO: code sanity check
        }
    }
    
    public double getAveragePosition() {
        return 0; // TODO: implement this method
    }
}