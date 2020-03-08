package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;

public class ArmSubsystem extends ConditionalSubsystemBase {

    // TODO Change placeholder values to actual limits
    private static final int FORWARD_LIMIT_TICKS = 2000;
    private static final int REVERSE_LIMIT_TICKS = 570;

    private static final int acceptableError = 50;

    private static ArmSubsystem INSTANCE = new ArmSubsystem();

    // TODO Change placeholder value to robotSpecific
    WPI_TalonSRX armTalon;
    ErrorCode errorCode;

    private static final int[] setpoints = {
            1500,
            3000,
        };


    private ArmSubsystem() {

        // Init all the talon values
        armTalon = new WPI_TalonSRX(Config.ARM_TALON);

        // Config factory default to clear out any lingering values
        armTalon.configFactoryDefault();

        // Allow the arm to be moved easily when disabled
        armTalon.setNeutralMode(NeutralMode.Coast);

        // Setup the talon, recording the error code
        errorCode = armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
                0, Config.CAN_TIMEOUT_SHORT);

        SmartDashboard.putNumber("Arm Error Code", errorCode.value);

        armTalon.setInverted(Config.INVERT_ARM_TALON);

        /* Config the peak and nominal outputs, 12V means full */
        armTalon.configNominalOutputForward(0, Config.CAN_TIMEOUT_SHORT);
        armTalon.configNominalOutputReverse(0, Config.CAN_TIMEOUT_SHORT);
        armTalon.configPeakOutputForward(1, Config.CAN_TIMEOUT_SHORT);
        armTalon.configPeakOutputReverse(-1, Config.CAN_TIMEOUT_SHORT);

        armTalon.configAllowableClosedloopError(0, Config.ARM_ALLOWABLE_CLOSED_LOOP_ERROR_TICKS, Config.CAN_TIMEOUT_SHORT);

        //  Config the PID Values based on constants
        armTalon.config_kP(0, Config.ARM_PID_P, Config.CAN_TIMEOUT_SHORT);
        armTalon.config_kI(0, Config.ARM_PID_I, Config.CAN_TIMEOUT_SHORT);
        armTalon.config_kD(0, Config.ARM_PID_D, Config.CAN_TIMEOUT_SHORT);


        // Set up the close loop period
        armTalon.configClosedLoopPeriod(0, Config.CAN_TIMEOUT_LONG);
        armTalon.setSensorPhase(true);
        armTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Config.CAN_TIMEOUT_LONG);
        armTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_TIMEOUT_LONG);

        //    Enable forward soft limit and set the value in encoder ticks
        armTalon.configForwardSoftLimitEnable(true);
        armTalon.configForwardSoftLimitThreshold(FORWARD_LIMIT_TICKS, Config.CAN_TIMEOUT_LONG);

        armTalon.configReverseSoftLimitEnable(true);
        armTalon.configReverseSoftLimitThreshold(REVERSE_LIMIT_TICKS, Config.CAN_TIMEOUT_SHORT);
        
        // Max voltage to apply with the talon. 12 is the maximum
        armTalon.configVoltageCompSaturation(12, Config.CAN_TIMEOUT_LONG);
        armTalon.enableVoltageCompensation(true);

        // Number of seconds from 0 to full throttle
     //   armTalon.configOpenloopRamp(0.6, Config.CAN_TIMEOUT_LONG);


        createCondition("talonFunctional", SubsystemConditionStates.ALWAYS);

        SubsystemCondition talonErrorCondition = getCondition("talonFunctional");

        if (errorCode.value == 0) {
            talonErrorCondition.setState(true);
        }
    }

    /**
     * Return the singleton arm instance
     *
     * @return the Arm instance
     */
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Set the talon to 0 encoder ticks
     */
    public void zeroTalonEncoder() {
        armTalon.setSelectedSensorPosition(0);
    }

    public void setpoint(int setpointIndex) {
        if(setpointIndex < setpoints.length) {
            armTalon.set(ControlMode.Position, setpoints[setpointIndex]);
        } else {
            DriverStation.reportError("Invalid arm position [Array index out of bounds]", false);
        }
    }

    public void moveArm(double speed) {
        armTalon.set(speed);
    }
    
    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Arm Motor Ticks", armTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Angle", toDeg(armTalon.getSelectedSensorPosition()));
    }

    public boolean reachedSetpoint(int index) {
        return reachedPosition(setpoints[index]);
    }

    public boolean reachedPosition(int position) {
        return position - acceptableError <= armTalon.getSelectedSensorPosition() && position + acceptableError >= armTalon.getSelectedSensorPosition();
    }

    /**
     * @param units CTRE mag encoder sensor units
     * @return degrees rounded to tenths.
     */
    Double toDeg(int units) {
        double deg = units * 360.0 / 4096.0;

        /* truncate to 0.1 res */
        deg *= 10;
        deg = (int) deg;
        deg /= 10;

        return deg;
    }
}
