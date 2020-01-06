package frc.robot.subsystems;

/** 
 * This is the code for the power cell intake. It initializes motor controllers and has methods
 * for the intake.
 * @author 
 */
public class Intake {

    private static Intake instance;

    private Intake() {

    }

    public static Intake getInstance() {
        if (instance == null) {
            return new Intake();
        }
        return instance;
    }
}