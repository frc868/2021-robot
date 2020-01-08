/** 
 * This is the code for getting the robot off of the baseline 
 * at the start of Auton
 * @author 
 */
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain;

 public class Baseline {
     private double distance;
     
    private Baseline(double distance) {
        this.distance = distance;
    }

    private double final startPwr = 1; //TODO: untested
    private double final endPwr = 2; //TODO: untested

    public void execute(){
        Robot.drivetrain.driveStraight(distance, startPwr, endPwr);
    }
 }
 