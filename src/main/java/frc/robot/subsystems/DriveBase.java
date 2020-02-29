/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
    private static WPI_TalonSRX leftFrontTalon, leftRearTalon, rightFrontTalon, rightRearTalon, talon5plyboy;
    //Talons were made static so that their current levels can be checked in static Driverbase methods.

    //create variable to display motor current levels
    public static double motorCurrent;

    //create a variable that states if motor current is actively being limited:
    public static boolean motorLimitActive = false;

    boolean EN_MOTOR_CURRENT_LIM = true; //Enable/disable motor current limiting. Not currently intended to change unless code is redeployed.

    public boolean sensitiveSteering = false;

    private PigeonIMU _pidgey;

    public static FluidConstant<Double> DRIVETRAIN_SENSITIVE_MAX_SPEED = new FluidConstant<>("DrivetrainSensitiveMaxSpeed", 0.2)
            .registerToTable(Config.constantsTable);

    private DriveBase() { 
        // Initialize the talons
        leftFrontTalon = new WPI_TalonSRX(Config.LEFT_FRONT_TALON);
        leftRearTalon = new WPI_TalonSRX(Config.LEFT_REAR_TALON);
        rightFrontTalon = new WPI_TalonSRX(Config.RIGHT_FRONT_TALON);
        rightRearTalon = new WPI_TalonSRX(Config.RIGHT_REAR_TALON);

        SmartDashboard.putNumber("Right Front Talon", Config.RIGHT_FRONT_TALON);

        //Set up the current limiter for all motors (limits current SUPPLY).        Enabled? True/false /         Limit (A)         /Surge trigger level (A) /  Max Surge time (sec)
        leftFrontTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(EN_MOTOR_CURRENT_LIM, Config.CONTIN_CURRENT_AMPS, Config.PEAK_CURRENT_AMPS, Config.PEAK_TIME_SEC));
		leftRearTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(EN_MOTOR_CURRENT_LIM, Config.CONTIN_CURRENT_AMPS, Config.PEAK_CURRENT_AMPS, Config.PEAK_TIME_SEC));
		rightFrontTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(EN_MOTOR_CURRENT_LIM, Config.CONTIN_CURRENT_AMPS, Config.PEAK_CURRENT_AMPS, Config.PEAK_TIME_SEC));
		rightRearTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(EN_MOTOR_CURRENT_LIM, Config.CONTIN_CURRENT_AMPS, Config.PEAK_CURRENT_AMPS, Config.PEAK_TIME_SEC));
		        
        talon5plyboy = new WPI_TalonSRX(Config.TALON_5_PLYBOY);

        follow();

        robotDriveBase = new DifferentialDrive(leftFrontTalon, rightFrontTalon);

        var pigeonTalon = Config.robotSpecific(null, null, rightRearTalon, leftFrontTalon, leftRearTalon, talon5plyboy);
        if(pigeonTalon != null){
            _pidgey = new PigeonIMU (pigeonTalon);
            _pidgey.setFusedHeading(0.0, 30);
        }

    }

    public static double getMotorCurrent() {
        motorCurrent = DriveBase.leftFrontTalon.getSupplyCurrent();

        SmartDashboard.putNumber("MotorCurrent (FrontLeft)", motorCurrent);
        
        return(motorCurrent);
    }

    public static boolean isMotorLimitActive() {
        //Check if motor current limiting is active (is current draw over or at current limit)
        if (motorCurrent >= Config.CONTIN_CURRENT_AMPS) {
            motorLimitActive = true;
        }
        else {
            motorLimitActive = false;
        }

        //Regardless of whether motor current limiting is active or not, tell shuffleboard and return the result.
        SmartDashboard.putBoolean("MotorCurrentLimit T/F", motorLimitActive);

        return(motorLimitActive);
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
        leftRearTalon.follow(leftFrontTalon);
        rightRearTalon.follow(rightFrontTalon);
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
        leftRearTalon.stopMotor();
        rightRearTalon.stopMotor();
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
        leftRearTalon.setNeutralMode(mode);
        rightFrontTalon.setNeutralMode(mode);
        rightRearTalon.setNeutralMode(mode);

        brakeMode = brake;
    }

    /**
     * Configure the encoder standard for the talons
     */
    private void selectEncoderStandard() {
        leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftRearTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightRearTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftFrontTalon.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
        leftRearTalon.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
        rightFrontTalon.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
        rightRearTalon.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);

        SmartDashboard.putNumber("Left Front", leftFrontTalon.getDeviceID());
        SmartDashboard.putNumber("Left Back", leftRearTalon.getDeviceID());
        SmartDashboard.putNumber("Right Front", rightFrontTalon.getDeviceID());
        SmartDashboard.putNumber("Right Back", leftFrontTalon.getDeviceID());

    }

    /**
     * Reset the talons to factory default
     */
    private void resetTalons() {
        leftRearTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        leftFrontTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightFrontTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightRearTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);

        leftRearTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
        leftFrontTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
        rightRearTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
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
