//package ca.team2706.frc.robot.subsystems;
package frc.robot.subsystems;

//import frc.robot.Sendables;
//import frc.robot.SubsystemStatus;
//import frc.robot.config.Config;
//import frc.robot.logging.Log;
//import frc.robot.sensors.AnalogSelector;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import com.ctre.phoenix.motion.MotionProfileStatus;

import com.ctre.phoenix.motion.TrajectoryPoint;

import com.ctre.phoenix.motorcontrol.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.wpilibj.PWM;

//Not currently working since class "Subsystem" has not been established
//import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

/**
 * 
 * Subsystem that controls the driving of the robot as well as certain sensors
 * that are used for driving
 * 
 */

// public class DriveBase extends Subsystem {
    public class DriveBase {
    private static DriveBase currentInstance;

    public static DriveBase getInstance() {

        init();

        return currentInstance;

    }

    /**
     * 
     * Initializes a new drive base object.
     * 
     */

    public static SubsystemStatus init() {

        if (currentInstance == null) {

            currentInstance = new DriveBase();

        }

        return currentInstance.getStatus();

    }

    /**
     * 
     * The mode that the robot is driving with
     * 
     */

    private DriveMode driveMode;

    /**
     * 
     * Four drive Talons
     * 
     */

    private final WPI_TalonSRX leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    /**
     * 
     * Logic for controlling robot motion in teleop
     * 
     */

    private final DifferentialDrive robotDriveBase;

    /**
     * 
     * Gyro to record robot heading
     * 
     */

    private final PigeonIMU gyro;

    /**
     * 
     * Analog Selector
     * 
     */

    private final AnalogSelector selector;

    /*
     * 
     * Purple light that goes on the robot
     * 
     */

    private final PWM light;

    /**
     * 
     * Indicates whether the robot is in brake mode
     * 
     */

    private boolean brakeMode;

    /**
     * 
     * Saves the absolute heading when the gyro is reset so that it can be
     * calculated from the relative angle
     * 
     */

    private double savedAngle;

    /**
     * 
     * Logs data to SmartDashboard and files periodically
     * 
     */

    private Notifier loggingNotifier;

    /**
     * 
     * Holds the status for the motion profile
     * 
     */

    private MotionProfileStatus motionProfileStatus;

    /**
     * 
     * Holds the points to stream for the right motion profile
     * 
     */

    private BufferedTrajectoryPointStream motionProfilePointStreamRight;

    /**
     * 
     * Holds the points to stream for the left motion profile
     * 
     */

    private BufferedTrajectoryPointStream motionProfilePointStreamLeft;

    private final SubsystemStatus status;

    /**
     * 
     * Creates a drive base, and initializes all required sensors and motors
     * 
     */

    private DriveBase() {

        leftFrontMotor = new WPI_TalonSRX(Config.LEFT_FRONT_DRIVE_MOTOR_ID);

        leftBackMotor = new WPI_TalonSRX(Config.LEFT_BACK_DRIVE_MOTOR_ID);

        rightFrontMotor = new WPI_TalonSRX(Config.RIGHT_FRONT_DRIVE_MOTOR_ID);

        rightBackMotor = new WPI_TalonSRX(Config.RIGHT_BACK_DRIVE_MOTOR_ID);

        SubsystemStatus status1 = resetTalonConfiguration();

        follow();

        enableCurrentLimit(Config.ENABLE_DRIVEBASE_CURRENT_LIMIT);

        robotDriveBase = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

        robotDriveBase.setRightSideInverted(false);

        robotDriveBase.setSafetyEnabled(false);

        gyro = new PigeonIMU(new TalonSRX(Config.GYRO_TALON_ID));

        selector = new AnalogSelector(Config.SELECTOR_ID);

        light = new PWM(Config.PURPLE_LIGHT);

        light.setRaw(4095);

        addChild("Left Front Motor", leftFrontMotor);

        addChild("Left Back Motor", leftBackMotor);

        addChild("Right Front Motor", rightFrontMotor);

        addChild("Right Back Motor", rightBackMotor);

        addChild(robotDriveBase);

        addChild("Gyroscope", Sendables.newPigeonSendable(gyro));

        addChild("Selector", selector);

        addChild("Left Encoder", Sendables.newTalonEncoderSendable(leftFrontMotor));

        addChild("Right Encoder", Sendables.newTalonEncoderSendable(rightFrontMotor));

        addChild("Merge Light", light);

        SubsystemStatus status2 = testSensors();

        setDisabledMode();

        setBrakeMode(false);

        loggingNotifier = new Notifier(this::log);

        loggingNotifier.startPeriodic(Config.LOG_PERIOD);

        motionProfileStatus = new MotionProfileStatus();

        motionProfilePointStreamLeft = new BufferedTrajectoryPointStream();

        motionProfilePointStreamRight = new BufferedTrajectoryPointStream();

        resetAbsoluteGyro();

        status = SubsystemStatus.maxError(status1, status2);

    }

    /**
     * 
     * Gets the subsystem's initialization status (status of sensors and systems).
     *
     * 
     * 
     * @return The subsystem's status
     * 
     */

    private SubsystemStatus getStatus() {

        return status;

    }

    /**
     * 
     * Whether there are errors in initialization that should disable autonomous
     *
     * 
     * 
     * @return Whether autonomous can run
     * 
     */

    private boolean canRunAuto() {

        return status == SubsystemStatus.OK || status == SubsystemStatus.WORKABLE;

    }

    /**
     * 
     * Tests the drive base sensors to determine if the subsystem is working as
     * expected.
     *
     * 
     * 
     * @return The subsystem's status based on the sensor test results.
     * 
     */

    private SubsystemStatus testSensors() {

        SubsystemStatus subsystemStatus = SubsystemStatus.OK;

        if (SubsystemStatus
                .checkError(leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative))) {

            Log.e("Left encoder not working");

            subsystemStatus = SubsystemStatus.DISABLE_AUTO;

        }

        if (SubsystemStatus
                .checkError(rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative))) {

            Log.e("Right encoder not working");

            subsystemStatus = SubsystemStatus.DISABLE_AUTO;

        }

        if (SubsystemStatus.checkError(gyro.getYawPitchRoll(new double[3]))) {

            Log.e("Gyro not working");

            subsystemStatus = SubsystemStatus.DISABLE_AUTO;

        }

        return subsystemStatus;

    }

    /**
     * 
     * Resets the talon configuration back to the initial config.
     * 
     */

    private SubsystemStatus resetTalonConfiguration() {

        SubsystemStatus status1 = SubsystemStatus.OK, status2 = SubsystemStatus.OK, status3 = SubsystemStatus.OK,
                status4 = SubsystemStatus.OK;

        if (SubsystemStatus.checkError(leftFrontMotor.configFactoryDefault(Config.CAN_LONG))) {

            Log.e("Can't reset left front motor to factory default");

            status1 = SubsystemStatus.ERROR;

        }

        if (SubsystemStatus.checkError(leftBackMotor.configFactoryDefault(Config.CAN_LONG))) {

            Log.e("Can't reset left back motor to factory default");

            status2 = SubsystemStatus.DISABLE_AUTO;

        }

        if (SubsystemStatus.checkError(rightFrontMotor.configFactoryDefault(Config.CAN_LONG))) {

            Log.e("Can't reset right front motor to factory default");

            status3 = SubsystemStatus.ERROR;

        }

        if (SubsystemStatus.checkError(rightBackMotor.configFactoryDefault(Config.CAN_LONG))) {

            Log.e("Can't reset right back motor to factory default");

            status4 = SubsystemStatus.DISABLE_AUTO;

        }

        leftFrontMotor.configPeakCurrentLimit(2, Config.CAN_LONG);

        leftBackMotor.configPeakCurrentLimit(2, Config.CAN_LONG);

        rightFrontMotor.configPeakCurrentLimit(2, Config.CAN_LONG);

        rightBackMotor.configPeakCurrentLimit(2, Config.CAN_LONG);

        leftFrontMotor.setInverted(Config.INVERT_FRONT_LEFT_DRIVE);

        rightFrontMotor.setInverted(Config.INVERT_FRONT_RIGHT_DRIVE);

        setTalonInversion(InvertType.FollowMaster, leftBackMotor, Config.INVERT_FRONT_LEFT_DRIVE,
                Config.INVERT_BACK_LEFT_DRIVE);

        setTalonInversion(InvertType.FollowMaster, rightBackMotor, Config.INVERT_FRONT_RIGHT_DRIVE,
                Config.INVERT_BACK_RIGHT_DRIVE);

        return SubsystemStatus.maxError(status1, status2, status3, status4);

    }

    /**
     * 
     * Gets the analog selector's index
     *
     * 
     * 
     * @return The index from 1-12 or 0 if unplugged
     * 
     */

    public int getAnalogSelectorIndex() {

        return selector.getIndex();

    }

    /**
     * 
     * Gets the drivemode that the robot is currently in
     *
     * 
     * 
     * @return The current drive mode
     * 
     */

    public DriveMode getDriveMode() {

        return driveMode;

    }

    /**
     * 
     * Stops the robot
     * 
     */

    public void stop() {

        leftFrontMotor.stopMotor();

        rightFrontMotor.stopMotor();

    }

    /**
     * 
     * Selects local encoders and the current sensor
     * 
     */

    private void selectEncodersStandard() {

        leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftFrontMotor.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND.value());

        rightFrontMotor.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND.value());

        leftBackMotor.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND.value());

        rightBackMotor.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND.value());

    }

    /**
     * 
     * Selects local encoders and the current sensor
     * 
     */

    private void selectEncodersSum() {

        resetTalonConfiguration();

        leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Config.CAN_SHORT);

        rightFrontMotor.configRemoteFeedbackFilter(leftFrontMotor.getDeviceID(),
                RemoteSensorSource.TalonSRX_SelectedSensor, 0, Config.CAN_SHORT);

        rightFrontMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Config.CAN_SHORT);

        rightFrontMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackCoefficient(0.5, 0, Config.CAN_SHORT);

        leftFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_LEFT.value());

        rightFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_RIGHT.value());

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Config.CAN_SHORT);

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_SHORT);

        leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Config.CAN_SHORT);

        configDeadband(leftFrontMotor, rightFrontMotor);

        rightFrontMotor.config_kP(0, Config.DRIVE_CLOSED_LOOP_P.value());

        rightFrontMotor.config_kI(0, Config.DRIVE_CLOSED_LOOP_I.value());

        rightFrontMotor.config_kD(0, Config.DRIVE_CLOSED_LOOP_D.value());

        rightFrontMotor.configClosedLoopPeriod(0, 1, Config.CAN_SHORT);

    }

    /**
     * 
     * Selects local encoders, the current sensor and the pigeon
     * 
     */

    private void selectEncodersSumWithPigeon(boolean motionProfile) {

        resetTalonConfiguration();

        leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Config.CAN_SHORT);

        rightFrontMotor.configRemoteFeedbackFilter(leftFrontMotor.getDeviceID(),
                RemoteSensorSource.TalonSRX_SelectedSensor, 0, Config.CAN_SHORT);

        rightFrontMotor.configRemoteFeedbackFilter(gyro.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 1,
                Config.CAN_SHORT);

        rightFrontMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Config.CAN_SHORT);

        rightFrontMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackCoefficient(0.5, 0, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackCoefficient(1, 1, Config.CAN_SHORT);

        leftFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_LEFT.value());

        rightFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_RIGHT.value());

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Config.CAN_SHORT);

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_SHORT);

        leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Config.CAN_SHORT);

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Config.CAN_SHORT);

        /* Configure neutral deadband */

        rightFrontMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value(), Config.CAN_SHORT);

        leftFrontMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value(), Config.CAN_SHORT);

        if (motionProfile) {

            rightFrontMotor.config_kP(0, Config.DRIVE_MOTION_MAGIC_P.value());

            rightFrontMotor.config_kI(0, Config.DRIVE_MOTION_MAGIC_I.value());

            rightFrontMotor.config_kD(0, Config.DRIVE_MOTION_MAGIC_D.value());

            rightFrontMotor.config_kF(0, Config.DRIVE_MOTION_MAGIC_F.value());

        } else {

            rightFrontMotor.config_kP(0, Config.DRIVE_CLOSED_LOOP_P.value());

            rightFrontMotor.config_kI(0, Config.DRIVE_CLOSED_LOOP_I.value());

            rightFrontMotor.config_kD(0, Config.DRIVE_CLOSED_LOOP_D.value());

            rightFrontMotor.config_kF(0, 0);

        }

        rightFrontMotor.config_kP(1, Config.PIGEON_KP.value());

        rightFrontMotor.config_kI(1, Config.PIGEON_KI.value());

        rightFrontMotor.config_kD(1, Config.PIGEON_KD.value());

        rightFrontMotor.config_kF(1, Config.PIGEON_KF.value());

        rightFrontMotor.configClosedLoopPeriod(0, 1, Config.CAN_SHORT);

        rightFrontMotor.configClosedLoopPeriod(1, 1, Config.CAN_SHORT);

        rightFrontMotor.configAuxPIDPolarity(false, Config.CAN_SHORT);

        rightFrontMotor.selectProfileSlot(0, 0);

        rightFrontMotor.selectProfileSlot(1, 1);

    }

    /**
     * 
     * Selects encoders for each wheel and configures the gyro as an auxiliary
     * sensor for each wheel
     * 
     */

    private void selectEncodersGyro(boolean motionProfile) {

        resetTalonConfiguration();

        leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Config.CAN_SHORT);

        rightFrontMotor.configRemoteFeedbackFilter(gyro.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 1,
                Config.CAN_SHORT);

        leftFrontMotor.configRemoteFeedbackFilter(gyro.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 1,
                Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, Config.CAN_SHORT);

        leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackCoefficient(1, 1, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackCoefficient(1, 0, Config.CAN_SHORT);

        leftFrontMotor.configSelectedFeedbackCoefficient(1, 1, Config.CAN_SHORT);

        leftFrontMotor.configSelectedFeedbackCoefficient(1, 0, Config.CAN_SHORT);

        leftFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_LEFT.value());

        rightFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_RIGHT.value());

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Config.CAN_SHORT);

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_SHORT);

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Config.CAN_SHORT);

        leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Config.CAN_SHORT);

        leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_SHORT);

        leftFrontMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Config.CAN_SHORT);

        /* Configure neutral deadband */

        rightFrontMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value(), Config.CAN_SHORT);

        leftFrontMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value(), Config.CAN_SHORT);

        if (motionProfile) {

            rightFrontMotor.config_kP(0, Config.DRIVE_MOTION_MAGIC_P.value());

            rightFrontMotor.config_kI(0, Config.DRIVE_MOTION_MAGIC_I.value());

            rightFrontMotor.config_kD(0, Config.DRIVE_MOTION_MAGIC_D.value());

            rightFrontMotor.config_kF(0, Config.DRIVE_MOTION_MAGIC_F.value());

            leftFrontMotor.config_kP(0, Config.DRIVE_MOTION_MAGIC_P.value());

            leftFrontMotor.config_kI(0, Config.DRIVE_MOTION_MAGIC_I.value());

            leftFrontMotor.config_kD(0, Config.DRIVE_MOTION_MAGIC_D.value());

            rightFrontMotor.config_kF(0, Config.DRIVE_MOTION_MAGIC_F.value());

        } else {

            rightFrontMotor.config_kP(0, Config.DRIVE_CLOSED_LOOP_P.value());

            rightFrontMotor.config_kI(0, Config.DRIVE_CLOSED_LOOP_I.value());

            rightFrontMotor.config_kD(0, Config.DRIVE_CLOSED_LOOP_D.value());

            rightFrontMotor.config_kF(0, 0);

            leftFrontMotor.config_kP(0, Config.DRIVE_CLOSED_LOOP_P.value());

            leftFrontMotor.config_kI(0, Config.DRIVE_CLOSED_LOOP_I.value());

            leftFrontMotor.config_kD(0, Config.DRIVE_CLOSED_LOOP_D.value());

            leftFrontMotor.config_kF(0, 0);

        }

        leftFrontMotor.config_kP(1, Config.PIGEON_KP.value());

        leftFrontMotor.config_kI(1, Config.PIGEON_KI.value());

        leftFrontMotor.config_kD(1, Config.PIGEON_KD.value());

        leftFrontMotor.config_kF(1, Config.PIGEON_KF.value());

        rightFrontMotor.config_kP(1, Config.PIGEON_KP.value());

        rightFrontMotor.config_kI(1, Config.PIGEON_KI.value());

        rightFrontMotor.config_kD(1, Config.PIGEON_KD.value());

        rightFrontMotor.config_kF(1, Config.PIGEON_KF.value());

        rightFrontMotor.configClosedLoopPeriod(0, 1, Config.CAN_SHORT);

        leftFrontMotor.configClosedLoopPeriod(0, 1, Config.CAN_SHORT);

        leftFrontMotor.configClosedLoopPeriod(1, 1, Config.CAN_SHORT);

        leftFrontMotor.configAuxPIDPolarity(true, Config.CAN_SHORT);

        rightFrontMotor.configClosedLoopPeriod(1, 1, Config.CAN_SHORT);

        rightFrontMotor.configAuxPIDPolarity(false, Config.CAN_SHORT);

        rightFrontMotor.selectProfileSlot(0, 0);

        rightFrontMotor.selectProfileSlot(1, 1);

        leftFrontMotor.selectProfileSlot(0, 0);

        leftFrontMotor.selectProfileSlot(1, 1);

    }

    /**
     * 
     * Configures the deadband for the two given motors.
     *
     * 
     * 
     * @param rightFrontMotor The talon object representing the right front motor.
     * 
     * @param leftFrontMotor  The talon object representing the left front motor.
     * 
     */

    private void configDeadband(WPI_TalonSRX rightFrontMotor, WPI_TalonSRX leftFrontMotor) {

        rightFrontMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value());

        leftFrontMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value());

        leftBackMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value());

        rightBackMotor.configNeutralDeadband(Config.DRIVE_CLOSED_LOOP_DEADBAND.value());

    }

    /**
     * 
     * Sets up talons for using the gyro sensor as feedback.
     * 
     */

    private void selectGyroSensor() {

        resetTalonConfiguration();

        rightFrontMotor.configRemoteFeedbackFilter(gyro.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 0,
                Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, Config.CAN_SHORT);

        rightFrontMotor.configSelectedFeedbackCoefficient(1.0, 0, Config.CAN_SHORT);

        leftFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_LEFT.value());

        rightFrontMotor.setSensorPhase(Config.DRIVE_SUM_PHASE_RIGHT.value());

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Config.CAN_SHORT);

        rightFrontMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_SHORT);

        configDeadband(rightFrontMotor, leftFrontMotor);

        rightFrontMotor.config_kP(0, Config.TURN_P.value());

        rightFrontMotor.config_kI(0, Config.TURN_I.value());

        rightFrontMotor.config_kD(0, Config.TURN_D.value());

        rightFrontMotor.configClosedLoopPeriod(0, 1, Config.CAN_SHORT);

        setTalonInversion(InvertType.OpposeMaster, leftFrontMotor, Config.INVERT_FRONT_RIGHT_DRIVE,
                Config.INVERT_FRONT_LEFT_DRIVE);

    }

    /**
     * 
     * Sets the talons to a disabled mode
     * 
     */

    public void setDisabledMode() {

        if (driveMode != DriveMode.Disabled) {

            resetTalonConfiguration();

            stop();

            reset();

            setDriveMode(DriveMode.Disabled);

        }

    }

    /**
     * 
     * Switches the talons to a mode that is optimal for driving the robot using
     * human input
     * 
     */

    public void setOpenLoopVoltageMode() {

        if (driveMode != DriveMode.OpenLoopVoltage) {

            stop();

            selectEncodersStandard();

            reset();

            setDriveMode(DriveMode.OpenLoopVoltage);

        }

    }

    /**
     * 
     * Sets the talons to a disabled mode
     * 
     */

    public void setPositionNoGyroMode() {

        if (!canRunAuto()) {

            setDisabledMode();

        } else if (driveMode != DriveMode.PositionNoGyro) {

            stop();

            selectEncodersSum();

            reset();

            setDriveMode(DriveMode.PositionNoGyro);

        }

    }

    /**
     * 
     * Sets the robot up for rotation.
     * 
     */

    public void setRotateMode() {

        if (!canRunAuto()) {

            setDisabledMode();

        } else if (driveMode != DriveMode.Rotate) {

            stop();

            selectGyroSensor();

            reset();

            setDriveMode(DriveMode.Rotate);

        }

    }

    /**
     * 
     * Sets motion magic
     * 
     */

    public void setMotionMagicWithGyroMode() {

        if (!canRunAuto()) {

            setDisabledMode();

        } else if (driveMode != DriveMode.MotionMagicWithGyro) {

            stop();

            selectEncodersSumWithPigeon(true);

            configMotionMagic();

            reset();

            driveMode = DriveMode.MotionMagicWithGyro;

        }

    }

    /**
     * 
     * Sets the drive mode to motion profile
     * 
     */

    public void setMotionProfile() {

        if (!canRunAuto()) {

            setDisabledMode();

        } else if (driveMode != DriveMode.MotionProfile) {

            stop();

            selectEncodersSumWithPigeon(true);

            configMotionProfile();

            reset();

            rightFrontMotor.startMotionProfile(motionProfilePointStreamRight, 20, ControlMode.MotionProfileArc);

            driveMode = DriveMode.MotionProfile;

        }

    }

    /**
     * 
     * Sets the drive mode to 2 wheel motion profile
     * 
     */

    public void setMotionProfile2Wheel() {

        if (!canRunAuto()) {

            setDisabledMode();

        } else if (driveMode != DriveMode.MotionProfile2Wheel) {

            stop();

            selectEncodersGyro(true);

            configMotionProfile();

            reset();

            rightFrontMotor.startMotionProfile(motionProfilePointStreamRight, 20, ControlMode.MotionProfileArc);

            leftFrontMotor.startMotionProfile(motionProfilePointStreamLeft, 20, ControlMode.MotionProfileArc);

            driveMode = DriveMode.MotionProfile2Wheel;

        }

    }

    /**
     * 
     * Configures motion magic
     * 
     */

    private void configMotionMagic() {

        rightFrontMotor.configMotionSCurveStrength(Config.DRIVEBASE_MOTION_MAGIC_SMOOTHING.value(), Config.CAN_SHORT);

        rightFrontMotor.configMotionCruiseVelocity(
                (int) (Config.DRIVEBASE_MOTION_MAGIC_CRUISE_VELOCITY.value() / Config.DRIVE_ENCODER_DPP / 10),
                Config.CAN_SHORT);

        rightFrontMotor.configMotionAcceleration(
                (int) (Config.DRIVEBASE_MOTION_MAGIC_ACCELERATION.value() / Config.DRIVE_ENCODER_DPP / 10),
                Config.CAN_SHORT);

    }

    /**
     * 
     * Configures motion profile
     * 
     */

    private void configMotionProfile() {

        rightFrontMotor.configMotionSCurveStrength(Config.DRIVEBASE_MOTION_MAGIC_SMOOTHING.value(), Config.CAN_SHORT);

        rightFrontMotor.configMotionCruiseVelocity(
                (int) (Config.DRIVEBASE_MOTION_MAGIC_CRUISE_VELOCITY.value() / Config.DRIVE_ENCODER_DPP / 10),
                Config.CAN_SHORT);

        rightFrontMotor.configMotionAcceleration(
                (int) (Config.DRIVEBASE_MOTION_MAGIC_ACCELERATION.value() / Config.DRIVE_ENCODER_DPP / 10),
                Config.CAN_SHORT);

        leftFrontMotor.configMotionSCurveStrength(Config.DRIVEBASE_MOTION_MAGIC_SMOOTHING.value(), Config.CAN_SHORT);

        leftFrontMotor.configMotionCruiseVelocity(
                (int) (Config.DRIVEBASE_MOTION_MAGIC_CRUISE_VELOCITY.value() / Config.DRIVE_ENCODER_DPP / 10),
                Config.CAN_SHORT);

        leftFrontMotor.configMotionAcceleration(
                (int) (Config.DRIVEBASE_MOTION_MAGIC_ACCELERATION.value() / Config.DRIVE_ENCODER_DPP / 10),
                Config.CAN_SHORT);

    }

    /**
     * 
     * Gets the encoder sum using the pigeon
     * 
     */

    public void setPositionGyroMode() {

        if (!canRunAuto()) {

            setDisabledMode();

        } else if (driveMode != DriveMode.PositionGyro) {

            stop();

            selectEncodersSumWithPigeon(false);

            reset();

            driveMode = DriveMode.PositionGyro;

        }

    }

    /**
     * 
     * Changes whether current limiting should be used
     *
     * 
     * 
     * @param enable Whether to enable the current or not
     * 
     */

    public void enableCurrentLimit(boolean enable) {

        leftFrontMotor.enableCurrentLimit(enable);

        leftBackMotor.enableCurrentLimit(enable);

        rightFrontMotor.enableCurrentLimit(enable);

        rightBackMotor.enableCurrentLimit(enable);

    }

    @Override

    protected void initDefaultCommand() {

    }

    /**
     * 
     * Has the slave motors follow the master motors
     * 
     */

    private void follow() {

        leftBackMotor.follow(leftFrontMotor);

        rightBackMotor.follow(rightFrontMotor);

    }

    /**
     * 
     * Checks whether the robot is in brake mode
     *
     * 
     * 
     * @return True when the Talons have the neutral mode set to
     *         {@code NeutralMode.Brake}
     * 
     */

    public boolean isBrakeMode() {

        return brakeMode;

    }

    /**
     * 
     * Changes whether the drive motors should coast or brake when output is 0
     *
     * 
     * 
     * @param brake Whether to turn on brake mode or not
     * 
     */

    public void setBrakeMode(boolean brake) {

        NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;

        leftFrontMotor.setNeutralMode(mode);

        leftBackMotor.setNeutralMode(mode);

        rightFrontMotor.setNeutralMode(mode);

        rightBackMotor.setNeutralMode(mode);

        brakeMode = brake;

    }

    /**
     * 
     * Drives the robot by controlling the speed of the left and right motors
     * individually
     *
     * 
     * 
     * @param leftSpeed     The speed for the left motors
     * 
     * @param rightSpeed    The speed for the right motors
     * 
     * @param squaredInputs Whether to square each of the values
     * 
     */

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {

        setOpenLoopVoltageMode();

        robotDriveBase.tankDrive(leftSpeed, rightSpeed, squaredInputs);

        follow();

    }

    /**
     * 
     * Drives the robot by controlling the forward and rotation values
     *
     * 
     * 
     * @param forwardSpeed  The amount to drive forward
     * 
     * @param rotateSpeed   The amount to rotate
     * 
     * @param squaredInputs Whether to square each of the values
     * 
     */

    public void arcadeDrive(double forwardSpeed, double rotateSpeed, boolean squaredInputs) {

        setOpenLoopVoltageMode();

        robotDriveBase.arcadeDrive(forwardSpeed, rotateSpeed, squaredInputs);

        follow();

    }

    /**
     * 
     * Drives the robot by controlling the forward and amount of curve values
     *
     * 
     * 
     * @param forwardSpeed The amount to drive forward
     * 
     * @param curveSpeed   The amount that the robot should curve while driving
     * 
     * @param override     When true will only use rotation values
     * 
     */

    public void curvatureDrive(double forwardSpeed, double curveSpeed, boolean override) {

        setOpenLoopVoltageMode();

        robotDriveBase.curvatureDrive(forwardSpeed, curveSpeed, override);

        follow();

    }

    /**
     * 
     * Goes to a position with the closed loop Talon PIDs using only encoder
     * information
     *
     * 
     * 
     * @param speed    The speed from 0 to 1
     * 
     * @param setpoint The setpoint to go to in feet
     * 
     */

    public void setPositionNoGyro(double speed, double setpoint) {

        setPositionNoGyroMode();

        leftFrontMotor.configClosedLoopPeakOutput(0, speed);

        rightFrontMotor.configClosedLoopPeakOutput(0, speed);

        rightFrontMotor.set(ControlMode.Position, setpoint / Config.DRIVE_ENCODER_DPP);

        leftFrontMotor.follow(rightFrontMotor);

        follow();

    }

    /**
     * 
     * Follows motion magic profile
     *
     * 
     * 
     * @param speed          The speed from 0 to 1
     * 
     * @param setpoint       The setpoint in feet
     * 
     * @param targetRotation The target to rotate to
     * 
     */

    public void setMotionMagicPositionGyro(double speed, double setpoint, double targetRotation) {

        setMotionMagicWithGyroMode();

        leftFrontMotor.configClosedLoopPeakOutput(0, speed);

        rightFrontMotor.configClosedLoopPeakOutput(0, speed);

        leftFrontMotor.configClosedLoopPeakOutput(1, speed);

        rightFrontMotor.configClosedLoopPeakOutput(1, speed);

        rightFrontMotor.set(ControlMode.MotionMagic, setpoint / Config.DRIVE_ENCODER_DPP, DemandType.AuxPID,
                targetRotation / Config.PIGEON_DPP);

        leftFrontMotor.follow(rightFrontMotor, FollowerType.AuxOutput1);

        follow();

    }

    /**
     * 
     * Runs the motion profile
     *
     * 
     * 
     * @param speed The speed from 0 to 1
     * 
     */

    public void runMotionProfile(final double speed) {

        setMotionProfile();

        configTalons(speed);

    }

    /**
     * 
     * Configures the talons for motion profiling.
     *
     * 
     * 
     * @param speed The speed, from 0 to 1.
     * 
     */

    private void configTalons(final double speed) {

        leftFrontMotor.configClosedLoopPeakOutput(0, speed);

        rightFrontMotor.configClosedLoopPeakOutput(0, speed);

        leftFrontMotor.configClosedLoopPeakOutput(1, speed);

        rightFrontMotor.configClosedLoopPeakOutput(1, speed);

        // Doing it automatically so don't disable

        rightFrontMotor.feed();

        leftFrontMotor.follow(rightFrontMotor, FollowerType.AuxOutput1);

    }

    /**
     * 
     * Runs the motion profile for each wheel
     *
     * 
     * 
     * @param speed The speed from 0 to 1
     * 
     */

    public void runMotionProfile2Wheel(final double speed) {

        setMotionProfile2Wheel();

        configTalons(speed);

        leftFrontMotor.feed();

    }

    /**
     * 
     * Applies the motion profile
     *
     * 
     * 
     * @param pos         The position of the robot at a trajectory point
     * 
     * @param vel         The velocity of the robot at a trajectory point
     * 
     * @param heading     The heading of the robot at a trajectory point
     * 
     * @param time        The time for each trajectory point
     * 
     * @param size        How many trajectories there are
     * 
     * @param talon       The talon
     * 
     * @param pointStream The point stream
     * 
     */

    private void pushMotionProfile(double[] pos, double[] vel, double[] heading, int[] time, int size,
            WPI_TalonSRX talon, BufferedTrajectoryPointStream pointStream) {

        /* create an empty point */

        TrajectoryPoint[] points = new TrajectoryPoint[size];

        /*
         * 
         * just in case we are interrupting another MP and there is still buffer
         * 
         * points in memory, clear it.
         * 
         */

        talon.clearMotionProfileTrajectories();

        pointStream.Clear();

        /*
         * set the base trajectory period to zero, use the individual trajectory period
         * below
         */

        talon.configMotionProfileTrajectoryPeriod(0, Config.CAN_SHORT);

        /* This is fast since it's just into our TOP buffer */

        for (int i = 0; i < size; ++i) {

            points[i] = new TrajectoryPoint();

            /* for each point, fill our structure and pass it to API */

            points[i].position = pos[i] / Config.DRIVE_ENCODER_DPP;

            points[i].velocity = vel[i] / Config.DRIVE_ENCODER_DPP / 10;

            points[i].auxiliaryPos = heading[i] / Config.PIGEON_DPP; /* scaled such that 3600 => 360 deg */

            points[i].headingDeg = heading[i];

            points[i].profileSlotSelect0 = 0;

            points[i].profileSlotSelect1 = 1;

            points[i].timeDur = time[i];

            points[i].zeroPos = i == 0;

            points[i].useAuxPID = true;

            points[i].auxiliaryArbFeedFwd = Config.CURVE_ADJUSTMENT.value() * vel[i]
                    / Config.DRIVEBASE_MOTION_MAGIC_CRUISE_VELOCITY.value();

            points[i].isLastPoint = (i + 1) == size;

        }

        pointStream.Write(points);

    }

    /**
     * 
     * Applies the motion profile for 1 wheel
     *
     * 
     * 
     * @param forwards Whether the robot is going forwards or not
     * 
     * @param pos      The position of the robot at a trajectory point
     * 
     * @param vel      The velocity of the robot at a trajectory point
     * 
     * @param heading  The heading of the robot at a trajectory point
     * 
     * @param time     The time for each trajectory point
     * 
     * @param size     How many trajectory points there are
     * 
     */

    public void pushMotionProfile1Wheel(boolean forwards, double[] pos, double[] vel, double[] heading, int[] time,
            int size) {

        pushMotionProfile(forwards ? pos : negateDoubleArray(pos), forwards ? vel : negateDoubleArray(vel), heading,
                time, size, rightFrontMotor, motionProfilePointStreamRight);

    }

    /**
     * 
     * Applies the motion profile for 2 wheels
     *
     * 
     * 
     * @param forwards Whether the robot is going forwards or not
     * 
     * @param posLeft  The position of the robot at a trajectory point for the left
     *                 wheel
     * 
     * @param velLeft  The velocity of the robot at a trajectory point for the left
     *                 wheel
     * 
     * @param heading  The heading of the robot at a trajectory point
     * 
     * @param time     The time for each trajectory point
     * 
     * @param size     How many trajectory points there are
     * 
     * @param posRight The position of the robot at a trajectory point for the right
     *                 wheel
     * 
     * @param velRight The velocity of the robot at a trajectory point for the right
     *                 wheel
     * 
     */

    public void pushMotionProfile2Wheel(boolean forwards, double[] posLeft, double[] velLeft, double[] heading,
            int[] time, int size, double[] posRight, double[] velRight) {

        pushMotionProfile(forwards ? posLeft : negateDoubleArray(posLeft),
                forwards ? velLeft : negateDoubleArray(velLeft), heading, time, size, leftFrontMotor,
                motionProfilePointStreamLeft);

        pushMotionProfile(forwards ? posRight : negateDoubleArray(posRight),
                forwards ? velRight : negateDoubleArray(velRight), heading, time, size, rightFrontMotor,
                motionProfilePointStreamRight);

    }

    /**
     * 
     * Makes all the elements in the double array negative
     *
     * 
     * 
     * @param array The array that is negated
     * 
     * @return The array
     * 
     */

    public static double[] negateDoubleArray(final double[] array) {

        return Arrays.stream(array)

                .map(operand -> -operand)

                .toArray();

    }

    /*
     * 
     * Sets the amount that the robot has to rotate.
     *
     * 
     * 
     * @param speed The speed of the rotation.
     * 
     * @param setpoint The setpoint (angle) to which the robot should rotate, in
     * degrees.
     * 
     */

    public void setRotation(double speed, double setpoint) {

        setRotateMode();

        leftFrontMotor.configClosedLoopPeakOutput(0, speed);

        rightFrontMotor.configClosedLoopPeakOutput(0, speed);

        rightFrontMotor.set(ControlMode.Position, setpoint / Config.PIGEON_DPP);

        leftFrontMotor.follow(rightFrontMotor);

        follow();

    }

    /**
     * 
     * Goes to a position with the closed loop Talon PIDs using only encoder and
     * gyro
     *
     * 
     * 
     * @param speed          The speed from 0 to 1
     * 
     * @param setpoint       The setpoint to go to in feet
     * 
     * @param targetRotation The desired rotation
     * 
     */

    public void setPositionGyro(double speed, double setpoint, double targetRotation) {

        setPositionGyroMode();

        leftFrontMotor.configClosedLoopPeakOutput(0, speed);

        rightFrontMotor.configClosedLoopPeakOutput(0, speed);

        leftFrontMotor.configClosedLoopPeakOutput(1, speed);

        rightFrontMotor.configClosedLoopPeakOutput(1, speed);

        rightFrontMotor.set(ControlMode.Position, setpoint / Config.DRIVE_ENCODER_DPP, DemandType.AuxPID,
                targetRotation);

        leftFrontMotor.follow(rightFrontMotor, FollowerType.AuxOutput1);

        follow();

    }

    /**
     * 
     * Get the distance travelled by the left encoder in feet
     *
     * 
     * 
     * @return The distance of the left encoder
     * 
     */

    public double getLeftDistance() {

        return leftFrontMotor.getSensorCollection().getQuadraturePosition() * Config.DRIVE_ENCODER_DPP;

    }

    /**
     * 
     * Get the distance travelled by the right encoder in feet
     *
     * 
     * 
     * @return The distance of the right encoder
     * 
     */

    public double getRightDistance() {

        return rightFrontMotor.getSensorCollection().getQuadraturePosition() * Config.DRIVE_ENCODER_DPP;

    }

    /**
     * 
     * Get the speed of the left encoder in feet per second
     *
     * 
     * 
     * @return The speed of the left encoder
     * 
     */

    public double getLeftSpeed() {

        return leftFrontMotor.getSensorCollection().getQuadratureVelocity() * Config.DRIVE_ENCODER_DPP * 10;

    }

    /**
     * 
     * Get the speed of the right encoder in feet per second
     *
     * 
     * 
     * @return The speed of the right encoder
     * 
     */

    public double getRightSpeed() {

        return rightFrontMotor.getSensorCollection().getQuadratureVelocity() * Config.DRIVE_ENCODER_DPP * 10;

    }

    /**
     * 
     * Gets the angle that the robot is facing in degrees
     *
     * 
     * 
     * @return The rotation of the robot
     * 
     */

    public double getHeading() {

        return Sendables.getPigeonYaw(gyro);

    }

    /**
     * 
     * Gets the angle that the robot is facing relative to when it was first powered
     * on
     *
     * 
     * 
     * @return The absolute rotation of the robot in degrees
     * 
     */

    public double getAbsoluteHeading() {

        return savedAngle + getHeading();

    }

    /**
     * 
     * Resets the encoder values to 0 ticks
     * 
     */

    public void resetEncoders() {

        leftFrontMotor.getSensorCollection().setQuadraturePosition(0, Config.CAN_SHORT);

        rightFrontMotor.getSensorCollection().setQuadraturePosition(0, Config.CAN_SHORT);

    }

    /**
     * 
     * Perform initializations so that the absolute gyro positon returned by
     * 
     * the getAbsoluteHeading() method is equal to the angle specified by
     * 
     * Config.ROBOT_START_ANGLE. The gyro measurement device will be reset to
     * 
     * 0 degrees.
     * 
     */

    public void resetAbsoluteGyro() {

        resetAbsoluteGyro(Config.ROBOT_START_ANGLE.value());

    }

    /**
     * 
     * Resets the absolute gyro to a certain angle
     *
     * 
     * 
     * @param savedAngle The angle from 0 to 360
     * 
     */

    public void resetAbsoluteGyro(double savedAngle) {

        this.savedAngle = savedAngle;

        gyro.setYaw(0, Config.CAN_SHORT);

    }

    /**
     * 
     * Resets the gyro to 0 degrees
     * 
     */

    public void resetGyro() {

        savedAngle = getAbsoluteHeading();

        gyro.setYaw(0, Config.CAN_SHORT);

    }

    /**
     * 
     * Resets the encoders and gyro
     * 
     */

    public void reset() {

        lastEncoderAv = 0;

        lastGyro = 0;

        resetEncoders();

        resetGyro();

    }

    /**
     * 
     * Gets the error for the left motor
     *
     * 
     * 
     * @return The error in feet
     * 
     */

    public double getLeftError() {

        return leftFrontMotor.getClosedLoopError(0) * Config.DRIVE_ENCODER_DPP;

    }

    /**
     * 
     * Gets the error for the right motor
     *
     * 
     * 
     * @return The error in feet
     * 
     */

    public double getRightError() {

        return rightFrontMotor.getClosedLoopError(0) * Config.DRIVE_ENCODER_DPP;

    }

    /**
     * 
     * Gets the current error on the pigeon (how far off target it is).
     *
     * 
     * 
     * @return Pigeon error in degrees.
     * 
     */

    public double getPigeonError() {

        return rightFrontMotor.getClosedLoopError(0) * Config.PIGEON_DPP;

    }

    /**
     * 
     * Returns if the motion profile for 1 wheel is finished
     * 
     */

    public boolean isFinishedMotionProfile() {

        return rightFrontMotor.isMotionProfileFinished();

    }

    /**
     * 
     * Returns if the motion profile for 2 wheels is finished
     *
     * 
     * 
     * @return If its finished or not
     * 
     */

    public boolean isFinishedMotionProfile2Wheel() {

        return (rightFrontMotor.isMotionProfileFinished() || leftFrontMotor.isMotionProfileFinished());

    }

    /**
     * 
     * Logs
     * 
     */

    public void log() {

        if (DriverStation.getInstance().isEnabled()) {

            Log.d("Relative Gyro: " + getHeading());

            Log.d("Absolute Gyro: " + getAbsoluteHeading());

            Log.d("Left front motor temperature: " + leftFrontMotor.getTemperature());

            Log.d("Right front motor temperature: " + rightFrontMotor.getTemperature());

            Log.d("Left back motor temperature: " + leftBackMotor.getTemperature());

            Log.d("Right back motor temperature: " + rightBackMotor.getTemperature());

            Log.d("Left front motor distance: "
                    + leftFrontMotor.getSensorCollection().getQuadraturePosition() / Config.DRIVE_ENCODER_DPP);

            Log.d("Right front motor distance: "
                    + rightFrontMotor.getSensorCollection().getQuadraturePosition() / Config.DRIVE_ENCODER_DPP);

            Log.d("Left back motor distance: "
                    + leftBackMotor.getSensorCollection().getQuadraturePosition() / Config.DRIVE_ENCODER_DPP);

            Log.d("Right back motor distance: "
                    + rightBackMotor.getSensorCollection().getQuadraturePosition() / Config.DRIVE_ENCODER_DPP);

            Log.d("Left front motor speed: "
                    + leftFrontMotor.getSensorCollection().getQuadratureVelocity() / Config.DRIVE_ENCODER_DPP * 10);

            Log.d("Right front motor speed: "
                    + rightFrontMotor.getSensorCollection().getQuadraturePosition() / Config.DRIVE_ENCODER_DPP * 10);

            Log.d("Left back motor speed: "
                    + leftBackMotor.getSensorCollection().getQuadraturePosition() / Config.DRIVE_ENCODER_DPP * 10);

            Log.d("Right back motor speed: "
                    + rightBackMotor.getSensorCollection().getQuadraturePosition() / Config.DRIVE_ENCODER_DPP * 10);

        }

        SmartDashboard.putNumber("Relative Gyro", getHeading());

        SmartDashboard.putNumber("Absolute Gyro", getAbsoluteHeading());

        SmartDashboard.putNumber("Left front motor temp", leftFrontMotor.getTemperature());

        SmartDashboard.putNumber("Right front motor temp", rightFrontMotor.getTemperature());

        SmartDashboard.putNumber("Left back motor temp", leftBackMotor.getTemperature());

        SmartDashboard.putNumber("Right back motor temp", rightBackMotor.getTemperature());

        SmartDashboard.putNumber("Left front motor distance: ",
                leftFrontMotor.getSensorCollection().getQuadraturePosition() * Config.DRIVE_ENCODER_DPP);

        SmartDashboard.putNumber("Right front motor distance: ",
                rightFrontMotor.getSensorCollection().getQuadraturePosition() * Config.DRIVE_ENCODER_DPP);

        SmartDashboard.putNumber("Left back motor distance: ",
                leftBackMotor.getSensorCollection().getQuadraturePosition() * Config.DRIVE_ENCODER_DPP);

        SmartDashboard.putNumber("Right back motor distance: ",
                rightBackMotor.getSensorCollection().getQuadraturePosition() * Config.DRIVE_ENCODER_DPP);

        SmartDashboard.putNumber("Left front motor speed",
                leftFrontMotor.getSensorCollection().getQuadratureVelocity() * Config.DRIVE_ENCODER_DPP * 10);

        SmartDashboard.putNumber("Right front motor speed",
                rightFrontMotor.getSensorCollection().getQuadratureVelocity() * Config.DRIVE_ENCODER_DPP * 10);

        SmartDashboard.putNumber("Left back motor speed",
                leftBackMotor.getSensorCollection().getQuadratureVelocity() * Config.DRIVE_ENCODER_DPP * 10);

        SmartDashboard.putNumber("Right back motor speed",
                rightBackMotor.getSensorCollection().getQuadratureVelocity() * Config.DRIVE_ENCODER_DPP * 10);

    }

    /**
     * 
     * Sets the current drive mode.
     *
     * 
     * 
     * @param driveMode The new drive mode.
     * 
     */

    private void setDriveMode(DriveMode driveMode) {

        this.driveMode = driveMode;

    }

    /**
     * 
     * Sets the inversion of the slave talon based on the constant for whether or
     * not those motors are inverted
     * 
     * by default.
     *
     * 
     * 
     * @param inversion        The type of desired inversion. Should be either
     *                         {@link InvertType#FollowMaster} or
     *                         {@link InvertType#OpposeMaster}
     * 
     * @param talon            The slave talon to be configured.
     * 
     * @param isMasterInverted True if the master talon motor is inverted by
     *                         default.
     * 
     * @param isSlaveInverted  True if the slave talon motor is inverted by default.
     * 
     */

    private static void setTalonInversion(final InvertType inversion, WPI_TalonSRX talon,
            final boolean isMasterInverted, final boolean isSlaveInverted) {

        if (isMasterInverted == isSlaveInverted) {

            /*
             * 
             * If both talons are of the same constant inversion and we want them to follow,
             * then they can just follow each other.
             * 
             * If both talons are of the same constant inversion and we want them to oppose,
             * then they can just oppose each other.
             * 
             */

            talon.setInverted(inversion);

        } else {

            if (inversion == InvertType.FollowMaster) {

                // If the talons are of opposite constant inversion and we want them to follow,
                // the slave should oppose the master.

                talon.setInverted(InvertType.OpposeMaster);

            } else {

                // If the talons are of opposite constant inversion and we want them to oppose,
                // the slave should follow the master.

                talon.setInverted(InvertType.FollowMaster);

            }

        }

    }

    private double lastEncoderAv = 0;

    private double lastGyro = 0;

    /**
     * 
     * The drive mode of the robot
     * 
     */

    public enum DriveMode {

        /**
         * 
         * There is no control mode active
         * 
         */

        Disabled,

        /**
         * 
         * Standard open loop voltage control
         * 
         */

        OpenLoopVoltage,

        /**
         * 
         * Performs closed loop position control without heading support
         * 
         */

        PositionNoGyro,

        /**
         * 
         * Motion magic with gyro
         * 
         */

        MotionMagicWithGyro,

        /**
         * 
         * Motion profile
         * 
         */

        MotionProfile,

        /**
         * 
         * Motion profile with 2 wheels
         * 
         */

        MotionProfile2Wheel,

        /**
         * 
         * Closed loop control with Auxiliary Pigeon Support
         * 
         */

        PositionGyro,

        /**
         * 
         * Rotates the robot with the gyroscope
         * 
         */

        Rotate

    }

}