package frc.robot.subsystems;

/** 
 * This is the code for the climber. It initializes motor controllers/pneumatics and has methods
 * for various functions of the climber.
 * @author 
 */
public class Climber {

    private static Climber instance;

    private Climber() {

    }

    public static Climber getInstance() {
        if (instance == null) {
            return new Climber();
        }
        return instance;
    }
}