package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_shooter;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;

  // PID values (currently set for protobot's shooter)
  public static FluidConstant<Double> P_SHOOTERSUBSYSTEM = new FluidConstant<>
    ("P_ShooterSubsystem", 0.0003).registerToTable(Config.constantsTable);

  public static FluidConstant<Double> I_SHOOTERSUBSYSTEM = new FluidConstant<>
    ("I_ShooterSubsystem", 0.0).registerToTable(Config.constantsTable);

  public static FluidConstant<Double> D_SHOOTERSUBSYSTEM = new FluidConstant<>
    ("D_ShooterSubsystem", 0.0001).registerToTable(Config.constantsTable);

  public static FluidConstant<Double> F_SHOOTERSUBSYSTEM = new FluidConstant<>
    ("F_ShooterSubsystem", 0.00018).registerToTable(Config.constantsTable);

  public static FluidConstant<Integer> SETPOINT_RPM = new FluidConstant<>
    ("setpointRPM", 0).registerToTable(Config.constantsTable);

  double kMaxOutput = 1; 
  double kMinOutput = -1;

  private final int RPM_TOLERANCE = 50;

  private ShooterSubsystem() {
    
    // Initialize a private variable for the motor
    if (Config.SHOOTER_MOTOR != -1){
      m_shooter = new CANSparkMax(Config.SHOOTER_MOTOR, MotorType.kBrushless);
    }

    // Factory Default to prevent unexpected behaviour
    m_shooter.restoreFactoryDefaults();

    // PID controller for the shooter
    m_pidController = m_shooter.getPIDController();
    m_encoder = m_shooter.getEncoder();

    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController.setFF(F_SHOOTERSUBSYSTEM.get());
    m_pidController.setP(P_SHOOTERSUBSYSTEM.get());
    m_pidController.setI(I_SHOOTERSUBSYSTEM.get());
    m_pidController.setD(D_SHOOTERSUBSYSTEM.get());
  }

  private static class ShooterHolder{
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
  public void setTargetRPM(int inputRPM){
    SETPOINT_RPM.setValue(inputRPM);
  }

  /**
   * Return the motor velocity (RPM) measured by the encoder
   */
  public double getRPM(){
    return m_encoder.getVelocity();
  }

  /**
   * Return the motor temperature (Celsius) as measured by the encoder
   */
  public double getTemperature(){
    return m_shooter.getMotorTemperature();
  }

  /**
   * Return the motor current draw measured by the encoder
   */
  public double getCurrentDraw(){
    return m_shooter.getOutputCurrent();
  }

  /**
   * Check the actual RPM and compare it with targetRPM
   * to verify that the shooter is up to necessary speed to fire.
   */
  public boolean isAtTargetRPM(){
    double encoderRPM = m_encoder.getVelocity();
    return (Math.abs(SETPOINT_RPM.get() - encoderRPM) < RPM_TOLERANCE);
      // I have no idea if +/- 50 tolerance around the RPM is accurate enough
      // Test and change, add setting conditions
  }

  @Override
  public void periodic() {
    if (m_shooter == null) return;
    m_pidController.setReference(SETPOINT_RPM.get(), ControlType.kVelocity);
  }
}