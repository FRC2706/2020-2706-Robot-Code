package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.sensors.AnalogSelector;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_shooter;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;

    public static FluidConstant<Double> P_SHOOTERSUBSYSTEM = new FluidConstant<>("P_ShooterSubsystem", 0.002)
        .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> I_SHOOTERSUBSYSTEM = new FluidConstant<>("I_ShooterSubsystem", 0.0)
        .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> D_SHOOTERSUBSYSTEM = new FluidConstant<>("D_ShooterSubsystem", 0.0002)
        .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> F_SHOOTERSUBSYSTEM = new FluidConstant<>("F_ShooterSubsystem", 0.00027)
        .registerToTable(Config.constantsTable);

    public static FluidConstant<Integer> SETPOINT_RPM = new FluidConstant<>("setpointRPM", 0)
        .registerToTable(Config.constantsTable);

    double kMaxOutput = 1;
    double kMinOutput = -1;

    private final int RPM_TOLERANCE = 30;

    private AnalogInput shooterAnalogSensor;

    public DigitalInput shooterDigitalInput;

    public Supplier<Boolean> isAtLimitSwitch;

    private ShooterSubsystem() {

        // Initialize the subsystem if the shooter exists
        if (Config.SHOOTER_MOTOR != -1) {
            initializeSubsystem();
        }
    }

    /**
     * Initialization process for the shooter to be run on robots with this
     * mechanism.
     */
    private void initializeSubsystem() {
        m_shooter = new CANSparkMax(Config.SHOOTER_MOTOR, MotorType.kBrushless);

        // Factory Default to prevent unexpected behaviour
        m_shooter.restoreFactoryDefaults();

        // PID controller for the shooter
        m_pidController = m_shooter.getPIDController();
        m_encoder = m_shooter.getEncoder();

        m_shooter.setInverted(true);

        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        m_pidController.setFF(F_SHOOTERSUBSYSTEM.get());
        m_pidController.setP(P_SHOOTERSUBSYSTEM.get());
        m_pidController.setI(I_SHOOTERSUBSYSTEM.get());
        m_pidController.setD(D_SHOOTERSUBSYSTEM.get());

        m_shooter.setSmartCurrentLimit(60);

        shooterAnalogSensor = new AnalogInput(2);

        shooterDigitalInput = new DigitalInput(8);

        isAtLimitSwitch = () -> shooterDigitalInput.get();


    }

    public boolean isActive() {
        return m_shooter != null;
    }

    private static class ShooterHolder {
        private static final ShooterSubsystem INSTANCE_SHOOTER = new ShooterSubsystem();
    }

    /**
     * Returns the singleton instance for the ShooterSubsystem
     */
    public static ShooterSubsystem getInstance() {
        return ShooterHolder.INSTANCE_SHOOTER;
    }

    /**
     * Set the target RPM to ramp up to.
     */
    public void setTargetRPM(int inputRPM) {
        SETPOINT_RPM.setValue(inputRPM);
    }

    /**
     * Return the motor velocity (RPM) measured by the encoder
     */
    public double getRPM() {
        return m_encoder.getVelocity();
    }

    /**
     * Return the motor temperature (Celsius) as measured by the encoder
     */
    public double getTemperature() {
        return m_shooter.getMotorTemperature();
    }

    /**
     * Return the motor current draw measured by the encoder
     */
    public double getCurrentDraw() {
        return m_shooter.getOutputCurrent();
    }

    /**
     * Check the actual RPM and compare it with targetRPM to verify that the shooter
     * is up to necessary speed to fire.
     */
    public boolean isAtTargetRPM() {
        double encoderRPM = m_encoder.getVelocity();
        return (Math.abs(SETPOINT_RPM.get() - encoderRPM) < RPM_TOLERANCE);
    }

    public int getAnalogSensorValue() {
        return shooterAnalogSensor.getValue();
    }

    @Override
    public void periodic() {
        if (m_shooter == null) {
            return;
        }
        if (SETPOINT_RPM.get() <= 0.0) {
            m_shooter.set(0.0);
        } else {
            m_pidController.setReference(SETPOINT_RPM.get(), ControlType.kVelocity);
        }

        SmartDashboard.putNumber("shooter RPM", m_encoder.getVelocity());
        SmartDashboard.putNumber("shooter RPM", m_encoder.getVelocity());
        SmartDashboard.putNumber("shooter temp", getTemperature());
        SmartDashboard.putNumber("shooter current", getCurrentDraw());
        SmartDashboard.putNumber("Shooter Analog Sensor", shooterAnalogSensor.getValue());
    }
  }