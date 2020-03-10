package frc.robot.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase2020;
import frc.robot.subsystems.DriveBasePre2020;
import frc.robot.subsystems.DriveBase;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.SimpleFormatter;

/**
 * Config manager for the robot
 */
public class Config {
    
    /**
     * Instructions for set up of robot.conf file on robot
     *
     * 0. Connect to the robot to the robot using a usb cable or the wifi network.
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
    
    public static FileHandler logFileHandler;
    
    static {
        try {
            String logFilename = new SimpleDateFormat("'Robotlog_'yyyy'-'MM'-'dd'_'HH'-'mm'-'ss'.txt'").format(new Date());
            logFileHandler = new FileHandler("/home/lvuser/logs/" + logFilename);
            SimpleFormatter formatter = new SimpleFormatter();
            logFileHandler.setFormatter(formatter);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * ID of the robot that code is running on
     */
    private static int robotId = -1;
    
    /**
     * PLACE IDS OF ROBOTS HERE
     **/
    // Mergio is has the ID 2
    // Mergonaut is ID 3
    
    // This is a static class which should not be instantiated
    private Config() {
    
    }
    
    // Static Constants
    private static Class<? extends DriveBase> Pre2020DriveBase = DriveBasePre2020.class.asSubclass(DriveBase.class);
    private static Class<? extends DriveBase> Post2020DriveBase = DriveBase2020.class.asSubclass(DriveBase.class);
    public static Class<? extends DriveBase> DRIVEBASE_CLASS = robotSpecific(Post2020DriveBase, Post2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase, Pre2020DriveBase);
    public static int RIGHT_FRONT_MOTOR = robotSpecific(2, 2, 3, 2, 2);
    public static int RIGHT_REAR_MOTOR = robotSpecific(4, 4, 4, 4, 4);
    public static int LEFT_FRONT_MOTOR = robotSpecific(1, 1, 1, 1, 1);
    public static int LEFT_REAR_MOTOR = robotSpecific(3, 3, 2, 3, 3);
    public static int INTAKE_MOTOR = robotSpecific(6, 6, -1, 6, -1);
    public static int SHOOTER_MOTOR = robotSpecific(5, 5, -1, -1, 16); //protobot is 16
    public static int CLIMBER_TALON = robotSpecific(10, 10, -1, -1, 16);
    public static int AGITATOR_MOTOR = robotSpecific(9, 9);

    // Current limiter Constants
    public static int PEAK_CURRENT_AMPS = 80;           //Peak current threshold to trigger the current limit
    public static int PEAK_TIME_MS = 250;               //Time after current exceeds peak current to trigger current limit
    public static int CONTIN_CURRENT_AMPS = 40;         //Current to mantain once current limit is triggered 
    public static boolean MOTOR_CURRENT_LIMIT = true;   //Enable or disable motor current limiting.

    public static int TALON_5_PLYBOY = robotSpecific(-1, -1, -1, -1, -1, 5);
    public static int PIGEON_ID = robotSpecific(CLIMBER_TALON, CLIMBER_TALON, RIGHT_REAR_MOTOR, LEFT_FRONT_MOTOR, LEFT_REAR_MOTOR, TALON_5_PLYBOY);
    
    public static int ANALOG_SELECTOR_ONE = robotSpecific(0, 0, -1, -1, -1, 0);
    
    public static int ARM_TALON = robotSpecific(7, 7, 12);

    public static int FEEDER_SUBSYSTEM_TALON = robotSpecific(8, 8);
    
    public static Double DRIVE_OPEN_LOOP_DEADBAND = 0.04;
    
    public static Double JOYSTICK_AXIS_DEADBAND = 0.1;
    
    public static int LEFT_CONTROL_STICK_Y = 1;
    public static int LEFT_CONTROL_STICK_X = 0;
    
    public static int RIGHT_CONTROL_STICK_Y = 5;
    public static int RIGHT_CONTROL_STICK_X = 4;
    
    public static boolean INVERT_FIRST_AXIS = robotSpecific(true, true, true);
    public static boolean INVERT_SECOND_AXIS = robotSpecific(false, false, true);
    
    public static double CONTROLLER_DEADBAND = 0.05;
    
    public static double CURVATURE_OVERRIDE = 0.25;

    public static boolean ARM_PHASE = robotSpecific(true, true);
    
    public static boolean INVERT_ARM_TALON = robotSpecific(true, true, false);
    
    public static int ARM_ALLOWABLE_CLOSED_LOOP_ERROR_TICKS = 4096;
    
    // Timeouts for sending CAN bus commands
    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;
    
    public static final boolean TELEOP_BRAKE = false;
    
    public static final boolean TELEOP_SQUARE_JOYSTICK_INPUTS = true;
    
    // PIDF values for the arm
    public static double ARM_PID_P = robotSpecific(5);
    public static double ARM_PID_I = robotSpecific(0.0);
    public static double ARM_PID_D = robotSpecific(0.0);
    public static double ARM_PID_F = robotSpecific(0.05);


    // Define a global constants table for subsystems to use
    public static NetworkTable constantsTable = NetworkTableInstance.getDefault().getTable("constants");

    // Vision Table Constants
    public static String VISION_TABLE_NAME_POWERCELL = "MergeVisionPi20";
    public static String VISION_TABLE_NAME_OUTERPORT = "MergeVisionPipelinePi21";
    public static String DISTANCE_POWERCELL     = "DistanceToPowerCell";
    public static String DISTANCE_OUTER_PORT    = "DistanceToOuterPort";
    public static String YAW_POWERCELL          = "YawToPowerCell";
    public static String YAW_OUTER_PORT         = "YawToTarget";

    // Drivetrain PID values
    public static double DRIVETRAIN_P_SPECIFIC = robotSpecific(0.037, 0.0, 0.0, 0.018d, 0.0, 0.25);
    public static double DRIVETRAIN_D_SPECIFIC = robotSpecific(0.0023, 0.0, 0.0, 0.0016d, 0.0, 0.03);

    public static final FluidConstant<Integer> RPM = new FluidConstant<>("Shooter RPM", 1700)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> DRIVETRAIN_P = new FluidConstant<>("DrivetrainP", DRIVETRAIN_P_SPECIFIC)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> DRIVETRAIN_D = new FluidConstant<>("DrivetrainD", DRIVETRAIN_D_SPECIFIC)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> maxTimeOuterPortCommand = new FluidConstant<>("Outer Port Max Time", 1.0)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> maxYawErrorOuterPortCommand = new FluidConstant<>("Outer Port Command Yaw Error", 3.0)
            .registerToTable(Config.constantsTable);
    
    // PID Values for the DrivetrainPIDTurnDelta command
    public static FluidConstant<Double> PIDTURNDELTA_P = new FluidConstant<>("DrivetrainP", 0.018d)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> PIDTURNDELTA_D = new FluidConstant<>("DrivetrainD", 0.0016d)
            .registerToTable(Config.constantsTable);

    // Fluid constant for Drivetrains
    public static FluidConstant<Double> DRIVETRAIN_SENSITIVE_MAX_SPEED = new FluidConstant<>("DrivetrainSensitiveMaxSpeed", 0.2)
            .registerToTable(Config.constantsTable);

    public static int shooterAnalogSensor = robotSpecific(8, 9);

    public static FluidConstant<Double> DRIVETRAIN_DEFAULT_MAX_SPEED = new FluidConstant<>("DrivetrainDefaultMaxSpeed", 1.0)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> FEEDERSUBSYSTEM_INCREMENT_TICKS = new FluidConstant<>("IncrementTicks", 1200.0)
            .registerToTable(Config.constantsTable);
    //Max distance at which the robot knows a ball is at the indexer
    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IR_MAX_DISTANCE = new FluidConstant<>("IrMaxDistance", 0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_P = new FluidConstant<>("FeederSubsystemP", 0.1)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_I = new FluidConstant<>("FeederSubsystemI", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_D = new FluidConstant<>("FeederSubsystemD", 0.05)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_F = new FluidConstant<>("FeederSubsystemF", 0.0)
                .registerToTable(Config.constantsTable);
    //Highest speed the motor could reach
    public static FluidConstant<Double> FEEDERSUBSYSTEM_PEAK_OUTPUT = new FluidConstant<>("FeederSubsystemPeakOutput", 0.35)
                .registerToTable(Config.constantsTable);
    
    /**
     * Returns one of the values passed based on the robot ID
     *
     * @param first The first value (default value)
     * @param more  Other values that could be selected
     * @param <T>   The type of the value
     * @return The value selected based on the ID of the robot
     */
    @SafeVarargs
    public static <T> T robotSpecific(T first, T... more) {
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
            } catch (Exception e) {
                Robot.haltRobot("Can't load Robot ID", e);
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
        if (value <= JOYSTICK_AXIS_DEADBAND && value >= 0) {
            return 0;
        } else if (value >= -JOYSTICK_AXIS_DEADBAND && value <= 0) {
            return 0;
        } else {
            return value;
        }
    }
}
