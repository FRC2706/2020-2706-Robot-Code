/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.config.Config;


public class DriveBasePre2020 extends DriveBase {
    // The drivebase talons
    private final WPI_TalonSRX leftFrontTalon, leftRearTalon, rightFrontTalon, rightRearTalon, talon5plyboy;
    
    public DriveBasePre2020() {

        resetMotors();
        
        // Initialize the talons
        leftFrontTalon = new WPI_TalonSRX(Config.LEFT_FRONT_MOTOR);
        leftRearTalon = new WPI_TalonSRX(Config.LEFT_REAR_MOTOR);
        rightFrontTalon = new WPI_TalonSRX(Config.RIGHT_FRONT_MOTOR);
        rightRearTalon = new WPI_TalonSRX(Config.RIGHT_REAR_MOTOR);
        
        talon5plyboy = new WPI_TalonSRX(Config.TALON_5_PLYBOY);
        
        differentialDrive = new DifferentialDrive(leftFrontTalon, rightFrontTalon);
        
        
        var pigeonTalon = Config.robotSpecific(null, null, rightRearTalon, leftFrontTalon, leftRearTalon, talon5plyboy);
        if (pigeonTalon != null) {
            pigeon = new PigeonIMU(pigeonTalon);
            pigeon.setFusedHeading(0.0, 30);
        }
        
    }
    
    @Override
    protected void driveModeUpdated(DriveMode mode) {
        this.stopMotors();
        if (mode == DriveMode.OpenLoopVoltage) {
            this.selectEncoderStandard();
        } else if (mode == DriveMode.Disabled) {
            this.resetMotors();
        }
    }
    
    /**
     * Changes whether the drive motors should coast or brake when output is 0
     *
     * @param mode Whether to turn on brake mode or not
     */
    @Override
    public void neutralModeUpdated(NeutralMode mode) {
        leftFrontTalon.setNeutralMode(mode);
        leftRearTalon.setNeutralMode(mode);
        rightFrontTalon.setNeutralMode(mode);
        rightRearTalon.setNeutralMode(mode);
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
        
    }
    
    /**
     * Reset the talons to factory default
     */
    @Override
    protected void resetMotors() {
        leftRearTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        leftFrontTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightFrontTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightRearTalon.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        
        leftRearTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
        leftFrontTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
        rightRearTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
        rightFrontTalon.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
    }
    
    @Override
    public void stopMotors() {
        leftRearTalon.stopMotor();
        leftFrontTalon.stopMotor();
        rightRearTalon.stopMotor();
        rightFrontTalon.stopMotor();
    }
    
    @Override
    protected void followMotors() {
        leftRearTalon.follow(leftFrontTalon);
        rightRearTalon.follow(rightFrontTalon);
    }
}
