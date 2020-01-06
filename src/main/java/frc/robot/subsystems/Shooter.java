package frc.robot.subsystems;

/** 
 * This is the code for the shooter. It initializes motor controllers and has methods
 * for various functions of the shooter. It also uses PID control for maintaining optimal velocity.
 * @author 
 */
public class Shooter {
    private static Shooter instance;

    private Shooter() {

    }

    public static Shooter getInstance() {
        if (instance == null) {
            return new Shooter();
        }
        return instance;
    }
}