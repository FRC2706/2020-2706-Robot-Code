package frc.robot.config;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class Config {

    /**
     * Instructions for set up of robot.conf file on robot
     * <p>
     * 1. Using a tool like Git Bash or putty, ssh into admin@roboRIO-2706-FRC.local (ssh admin@roboRIO-2706-FRC.local)
     * a. There is no password on a freshly flashed roboRIO
     * 2. Go up a directory (cd ..)
     * 3. cd into lvuser/ (cd lvuser/)
     * 4. Create a new file called robot.conf (touch robot.conf)
     * 5. Open the file with vi (vi robot.conf)
     * 6. Press i to enter insert mode
     * 7. Add an integer denoting the robot id. If it's the first robot, use 0, second use 1 etc.
     * 8. Press [ESC] followed by typing :wq in order to save and quit
     * 9. To verify this worked type: more robot.conf
     * 10. If it displays the value you entered, it was successful
     * 11. Type exit to safely exit the ssh session
     */

    private static final Path ROBOT_ID_LOC = Paths.get(System.getProperty("user.home"), "robot.conf");

    /**
     * ID of the robot that code is running on
     */
    private static int robotId = -1;

    /**
     * PLACE IDS OF ROBOTS HERE
     **/
    // Mergio is has the ID 2

    // This is a static class which should not be instantiated
    private Config() {

    }

    // Static Constants
    public static int RIGHT_FRONT_TALON = robotSpecific(3, 3, 3);
    public static int RIGHT_REAR_TALON = robotSpecific(4, 4, 4);
    public static int LEFT_FRONT_TALON = robotSpecific(1, 1, 1);
    public static int LEFT_REAR_TALON = robotSpecific(2, 2, 2);

    public static int ARM_TALON = robotSpecific(12, 12, 12);

    public static Double DRIVE_OPEN_LOOP_DEADBAND = 0.04;

    public static Double JOYSTICK_AXIS_DEADBAND = 0.1;

    public static int LEFT_CONTROL_STICK_Y = 1;
    public static int LEFT_CONTROL_STICK_X = 0;

    public static int RIGHT_CONTROL_STICK_Y = 5;
    public static int RIGHT_CONTROL_STICK_X = 4;

    public static boolean INVERT_FORWARD = robotSpecific(true, true, true);
    public static boolean INVERT_SIDE= robotSpecific(false, false, false);

    public static boolean INVERT_ARM_TALON = robotSpecific(false, false, false);

    public static int ARM_ALLOWABLE_CLOSED = 4096;

    // Timeouts for sending CAN bus commands
    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;

    // PIDF values for the arm
    public static double ARM_P = robotSpecific(0.2);
    public static double ARM_I = robotSpecific(0.0);
    public static double ARM_D = robotSpecific(0.1);
    public static double ARM_F = robotSpecific(0.0);

    /**
     * Returns one of the values passed based on the robot ID
     *
     * @param first The first value (default value)
     * @param more  Other values that could be selected
     * @param <T>   The type of the value
     * @return The value selected based on the ID of the robot
     */
    @SafeVarargs
    private static <T> T robotSpecific(T first, T... more) {
        if (getRobotId() < 1 || getRobotId() > more.length) {
            return first;
        } else {
            return more[getRobotId() - 1];
        }
    }


    /**
     * Obtain the robot id found in the robot.conf file
     *
     * @return The id of the robot
     */
    private static int getRobotId() {
        if (robotId < 0) {
            try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
                robotId = Integer.parseInt(reader.readLine());
            } catch (IOException | NumberFormatException e) {
                robotId = 0;
                DriverStation.reportError("Could not find robot configuration file.", false);
            }
        }
        return robotId;
    }

    /**
     *
     * @param value The raw axis value from the control stick
     * @return The filtered value defined by the acceptable dead band
     */
    public static double removeJoystickDeadband(double value) {
        if(value <= JOYSTICK_AXIS_DEADBAND && value >= 0) {
            return 0;
        } else if(value >= -JOYSTICK_AXIS_DEADBAND && value <= 0) {
            return 0;
        } else {
            return value;
        }
    }

}
