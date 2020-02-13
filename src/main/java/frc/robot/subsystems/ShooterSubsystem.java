package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.config.Config;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_shooter;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;

  double kF = 0;
  double kP = 0; 
  double kI = 0;
  double kD = 0;

  int setpointRPM;

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

    m_pidController.setFF(kF);
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
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
  public void setRPM(int inputRPM){
    setpointRPM = inputRPM;
  }

  /**
   * Return the motor velocity measured by the encoder
   */
  public double getRPM(){
    double encoderRPM = m_encoder.getVelocity();
    return encoderRPM;
  }

  /**
   * Check the actual RPM and compare it with targetRPM
   * to verify that the shooter is up to necessary speed to fire.
   */
  public boolean checkRPM(int inputRPM){
    double encoderRPM = m_encoder.getVelocity();
    return (encoderRPM < (inputRPM + RPM_TOLERANCE) && encoderRPM > (inputRPM - RPM_TOLERANCE));
      // I have no idea if +/- 50 tolerance around the RPM is accurate enough
      // Test and change, add setting conditions
  }

  @Override
  public void periodic() {
    if (m_shooter == null) return;
    m_pidController.setReference(setpointRPM, ControlType.kVelocity);
  }
}