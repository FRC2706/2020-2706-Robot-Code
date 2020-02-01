package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class Arm extends SubsystemBase {

    private static final int forwardLimit = 100000;
    private static final int reverseLimit = -10000;

    private static Arm currentInstance;
    // TODO Change placeholder value to robotSpecific
    WPI_TalonSRX armTalon;
    ErrorCode errorCode;

    private Arm() {
        // Init all the talon values
        armTalon = new WPI_TalonSRX(12);

        armTalon.configFactoryDefault();

        armTalon.setNeutralMode(NeutralMode.Coast);

        errorCode = armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                0, Config.CAN_TIMEOUT_SHORT);

        armTalon.setSelectedSensorPosition(0);

        armTalon.setInverted(Config.INVERT_ARM_TALON);

        /* Config the peak and nominal outputs, 12V means full */
        armTalon.configNominalOutputForward(0, Config.CAN_TIMEOUT_SHORT);
        armTalon.configNominalOutputReverse(0, Config.CAN_TIMEOUT_SHORT);
        armTalon.configPeakOutputForward(1, Config.CAN_TIMEOUT_SHORT);
        armTalon.configPeakOutputReverse(-1, Config.CAN_TIMEOUT_SHORT);

         armTalon.configAllowableClosedloopError(0, Config.ARM_ALLOWABLE_CLOSED, Config.CAN_TIMEOUT_SHORT);


            //  armTalon.configSelectedFeedbackCoefficient(0.5, 0, Config.CAN_TIMEOUT_LONG);
       //  Config the PID Values
        armTalon.config_kP(0, Config.ARM_P, Config.CAN_TIMEOUT_SHORT);
        armTalon.config_kI(0, Config.ARM_I, Config.CAN_TIMEOUT_SHORT);
        armTalon.config_kD(0, Config.ARM_D, Config.CAN_TIMEOUT_SHORT);
        armTalon.config_kF(0, Config.ARM_F, Config.CAN_TIMEOUT_SHORT);

        armTalon.configClosedLoopPeriod(0, Config.CAN_TIMEOUT_LONG);
        armTalon.setSensorPhase(true);
        armTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Config.CAN_TIMEOUT_LONG);
        armTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_TIMEOUT_LONG);

        armTalon.configForwardSoftLimitEnable(true);
        armTalon.configForwardSoftLimitThreshold(forwardLimit, Config.CAN_TIMEOUT_LONG);

        armTalon.configReverseSoftLimitEnable(true);
        armTalon.configReverseSoftLimitThreshold(reverseLimit, Config.CAN_TIMEOUT_LONG);

        armTalon.configVoltageCompSaturation(12, Config.CAN_TIMEOUT_LONG);
        armTalon.enableVoltageCompensation(true);

        armTalon.configOpenloopRamp(0.6, Config.CAN_TIMEOUT_LONG);

    }

    public static void init() {
        if (currentInstance == null) {
            currentInstance = new Arm();
        }
    }

    public static Arm getInstance() {
        init();
        return currentInstance;
    }

    public void moveDown() {
        armTalon.set(-0.2);
    }

    public void moveUp() {
        armTalon.configClosedLoopPeakOutput(0, 80);
        armTalon.set(ControlMode.Position, 2000);
    }

    public void stop() {
        armTalon.set(0);
    }

    public void zeroTalonEncoder() {
        armTalon.setSelectedSensorPosition(0);
    }


    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Arm Motor Ticks", armTalon.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Arm Talon Error Code", errorCode.value);
    }
}
