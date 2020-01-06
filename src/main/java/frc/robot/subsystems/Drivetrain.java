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
}