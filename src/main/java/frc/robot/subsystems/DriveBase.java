/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;


public class DriveBase extends SubsystemBase {

    // DriveBase is a singleton class as it represents a physical subsystem
    private static DriveBase currentInstance;

    // The way the robot drives
    private static DifferentialDrive robotDriveBase;

    /**
     * Indicates whether the robot is in brake mode
     */
    private boolean brakeMode;

    // The mode in which the robot drives
    private DriveMode driveMode;

    // The drivebase talons
    private WPI_TalonSRX leftFrontTalon, rightFrontTalon, talon5plyboy;

    // Will be moved later
    private WPI_TalonSRX climberTalon = new WPI_TalonSRX(Config.CLIMBER_TALON);

    private VictorSPX leftRearVictor, rightRearVictor;

    public boolean sensitiveSteering = false;

    private PigeonIMU _pidgey;

    public static FluidConstant<Double> DRIVETRAIN_SENSITIVE_MAX_SPEED = new FluidConstant<>("DrivetrainSensitiveMaxSpeed", 0.2)
            .registerToTable(Config.constantsTable);

    private DriveBase() {

        // Initialize the talons
        leftFrontTalon = new WPI_TalonSRX(Config.LEFT_FRONT_MOTOR);
        leftRearVictor = new VictorSPX(Config.LEFT_REAR_MOTOR);
        rightFrontTalon = new WPI_TalonSRX(Config.RIGHT_FRONT_MOTOR);
        rightRearVictor = new VictorSPX(Config.RIGHT_REAR_MOTOR);

        SmartDashboard.putNumber("Right Front Talon", Config.RIGHT_FRONT_MOTOR);

        talon5plyboy = new WPI_TalonSRX(Config.TALON_5_PLYBOY);

        follow();

        robotDriveBase = new DifferentialDrive(leftFrontTalon, rightFrontTalon);

        var pigeonTalon = Config.robotSpecific(climberTalon, null, rightRearVictor, leftFrontTalon, leftRearVictor, talon5plyboy);
        if(pigeonTalon != null) {
            // Hardcoded for testing
            _pidgey = new PigeonIMU (climberTalon);
            _pidgey.setFusedHeading(0.0, 30);
        }

    }

    /**
     * Initialize the current DriveBase instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new DriveBase();
        }
    }

    public static DriveBase getInstance() {
        init();
        return currentInstance;
    }

    /**
     * This just returns the pigeon
     * @return
     */
    public PigeonIMU getPigeon() {
        return _pidgey;
    }

    /**
     * Gets the current angle based upon the angle the robot was enabled on
     * @return returns angle in degrees
     */
    public double getCurrentAngle(){
        //Gets the current angle
        try{
            PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
            _pidgey.getFusedHeading(fusionStatus);
            return fusionStatus.heading;
        }
        catch(NullPointerException e){
            return (0);
        }
    }

    /**
     * Sets the talons to a disabled mode
     */
    public void setDisabledMode() {
        if (driveMode != DriveMode.Disabled) {
            resetTalons();
            stop();
            driveMode = DriveMode.Disabled;
        }
    }

    /**
     * Make the back talons follow the front talons
     */
    private void follow() {
        leftRearVictor.follow(leftFrontTalon);
        rightRearVictor.follow(rightFrontTalon);
    }

    /**
     * Have the robot drive using the built-in arcade drive
     *
     * @param forwardVal The speed at which to move forward, between -1 and 1
     * @param rotateVal The speed at which to rotate, between -1 (Turn Left) and 1 (Turn Right)
     * @param squareInputs Weather or not to square the inputs (makes driving less sensitive)
     */
    public void arcadeDrive(double forwardVal, double rotateVal, boolean squareInputs) {
        setOpenLoopVoltage();
        if (!sensitiveSteering){
            robotDriveBase.arcadeDrive(forwardVal, rotateVal, squareInputs);
        }
        follow();
    }

    /**
     * Stop all the talons
     */
    public void tankDrive(double leftVal, double rightVal, boolean squareInputs){
        setOpenLoopVoltage();
        //steers the robot at a much lower max speed if sensitive control is on
        if (sensitiveSteering){
            robotDriveBase.tankDrive(leftVal*DRIVETRAIN_SENSITIVE_MAX_SPEED.get(), -rightVal*DRIVETRAIN_SENSITIVE_MAX_SPEED.get(), squareInputs);
        } else {
            robotDriveBase.tankDrive(leftVal, rightVal, squareInputs);
        }
        
        follow();
    }

    /**
     * Stop all the talons
     */
    public void stop() {
        leftFrontTalon.stopMotor();
        rightFrontTalon.stopMotor();
    }

    /**
     * Set up open loop voltage.
     *
     * This is optimal for driving by a human
     */
    public void setOpenLoopVoltage() {
        if (driveMode != DriveMode.OpenLoopVoltage) {
            stop();
            selectEncoderStandard();

            driveMode = DriveMode.OpenLoopVoltage;
        }

    }

    /**
     * Changes whether the drive motors should coast or brake when output is 0
     *
     * @param brake Whether to turn on brake mode or not
     */
    public void setBrakeMode(boolean brake) {
        NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;

        leftFrontTalon.setNeutralMode(mode);
        leftRearVictor.setNeutralMode(mode);
        rightFrontTalon.setNeutralMode(mode);
        rightRearVictor.setNeutralMode(mode);

        brakeMode = brake;
    }

    /**
     * Configure the encoder standard for the talons
     */
    private void selectEncoderStandard() {
        leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftRearVictor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightRearVictor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftFrontTalon.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
        leftRearVictor.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
        rightFrontTalon.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
        rightRearVictor.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);

        SmartDashboard.putNumber("Left Front", leftFrontTalon.getDeviceID());
        SmartDashboard.putNumber("Left Back", leftRearVictor.getDeviceID());
        SmartDashboard.putNumber("Right Front", rightFrontTalon.getDeviceID());
        SmartDashboard.putNumber("Right Back", leftFrontTalon.getDeviceID());

    }

    /**
     * Reset the talons to factory default
     */
    private void resetTalons() {
        leftRearVictor.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        leftFrontTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightFrontTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightRearVictor.configFactoryDefault(Config.CAN_TIMEOUT_LONG);

        leftFrontTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
        rightFrontTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
    }


    /**
     * The drive mode of the robot
     */
    public enum DriveMode {
        /**
         * There is no control mode active
         */
        Disabled,

        /**
         * Standard open loop voltage control
         */
        OpenLoopVoltage

    }

    /**
     *  Standard Curve Drive
     */
	public void curvatureDrive(double forwardSpeed, double curveSpeed, boolean override) {
        setOpenLoopVoltage();
        robotDriveBase.curvatureDrive(forwardSpeed, curveSpeed, override);
        follow();
    }
    
     /**
     * Checks whether the robot is in brake mode
     *
     * @return True when the Talons have the neutral mode set to {@code NeutralMode.Brake}
     */
    public boolean isBrakeMode() {
        return brakeMode;
    }
}
