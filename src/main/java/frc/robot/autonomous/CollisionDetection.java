package frc.robot.autonomous;
/** 
 * Gets the robot to the baseline.
 * @author jk
 */

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;


/** 
 * This class is run automatically, and dictates what functions are run during each of these stages.
 * @author dri
 */
public class CollisionDetection {
    AHRS ahrs;

    double prevlinearAccelX;
    double prevlinearAccelY;
    double currentlinearAccelX;
    double currentlinearAccelY;
    
    final static double COLLISION_THRESHOLD = 0.5d;
    private static CollisionDetection instance;

    private CollisionDetection() {
        try {
            ahrs = new AHRS(RobotMap.CollisionDetection.NAVX);
        } catch (RuntimeException e) {
            System.out.println("Oopsie daisy");
        }
    }

    public static CollisionDetection getInstance() {
        if (instance == null) {
            return new CollisionDetection();
        }
        return instance;
    }

    /**
     * the linearAccelX to set
     */
    public boolean detectCollision() {
        boolean collisionDetected = false;

        currentlinearAccelX = ahrs.getWorldLinearAccelX();
        double currentJerkX = currentlinearAccelX - prevlinearAccelX;
        prevlinearAccelX = currentlinearAccelX;

        currentlinearAccelY = ahrs.getWorldLinearAccelY();
        double currentJerkY = currentlinearAccelY - prevlinearAccelY;
        prevlinearAccelY = currentlinearAccelY;

        if ( ( Math.abs(currentJerkX) > COLLISION_THRESHOLD ) ||
               ( Math.abs(currentJerkY) > COLLISION_THRESHOLD) ) {
              collisionDetected = true;
        }

        test(collisionDetected);

        return collisionDetected;
    }

    public void test(boolean testst) {
        System.out.println(testst);
    }
}
