package frc.robot.subsystems;

/** 
 * This is the code for the climber. It initializes motor controllers/pneumatics and has methods
 * for various functions of the climber.
 * @author 
 */
public class Climber {
    private static Climber instance;

    private Climber() {
        primary = new CANSparkMax(RobotMap.Climber.PRIMARY, MotorType.kBrushless);  //TODO: untested
    }

    public static Climber getInstance() {
        if (instance == null) {
            return new Climber();
        }
        return instance;
    }
    @Override
    public void robotInit() {
        initialPosition = primary.getEncoder().getPosition();
        primary.getEncoder().setPosition(0);
        primary.setInverted(false);
    }
    public void toSetpoint(double targetDist, double power) {
        double pGain = .1;
        double distanceToTarget = Math.abs(targetDist) - Math.abs(primary.getEncoder().getPosition() - initialPosition);
        double targetSpeed = pGain * (power * distanceToTarget);
        if (distanceToTarget > 0) {
            primary.set(targetSpeed); // TODO: code sanity check
        }
    }
}