package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.config.Config;

import java.lang.reflect.InvocationTargetException;

public class DriveBaseHolder {
    private static DriveBase INSTANCE;

    public static void init() {
        try {
            INSTANCE = Config.DRIVEBASE_CLASS.getDeclaredConstructor().newInstance();
        } catch (NoSuchMethodException | IllegalAccessException | InstantiationException | InvocationTargetException e) {
            Robot.haltRobot("Cannot instantiate instance of DriveBase", e);
        }
    }

    public static DriveBase getInstance() {
        return INSTANCE;
    }
}
