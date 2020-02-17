package frc.robot.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.paths.DoNothing;
import frc.robot.autonomous.paths.Baseline;
import frc.robot.autonomous.paths.HeadOn;
import frc.robot.autonomous.paths.TrenchRun;

/**
 * Adds and selects autonomous paths by a SendableChooser.
 *
 * @author hrl
 */
public class AutonHelper {
    private static AutonHelper instance;

    private HashMap<String, AutonPath> paths = new HashMap<String, AutonPath>();
    private SendableChooser<String> chooser = new SendableChooser<>();

    private AutonHelper() {
        this.addPath("Nothing", new DoNothing());
        this.addPath("Baseline", new Baseline());
        this.addPath("Head-On", new HeadOn());
        this.addPath("Trench Run", new TrenchRun());
    }

    /**
     * Returns a singleton instance of the auton helper.
     */
    public static AutonHelper getInstance() {
        if (instance == null) {
            return new AutonHelper();
        }

        return instance;
    }

    /**
     * Adds a path to the auton helper.
     * @param name the name to assign the autonomous path
     * @param func the autonomous path itself
     */
    public void addPath(String name, AutonPath func) {
        paths.put(name, func);
        chooser.addOption(name, name);
    }


    /**
     * Returns the current autonomous path's key.
     */
    public String getCurrentPath() {
        return chooser.getSelected();
    }

    /**
     * Runs whatever path is selected on the sendable chooser.
     */
    public void runSelectedPath() {
        paths.get(chooser.getSelected()).run();
    }

    /**
     * Puts the SendableChooser to the SmartDashboard.
     */
    public void initSD() {
        SmartDashboard.putData(chooser);
    }
}
