package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.config.Config;

public class TalonShooterSubsystem extends SubsystemBase {

  private WPI_TalonSRX m_shooter;

  // Indicates use of the primary PID loop rather than cascaded ones
  int kPIDLoopIDX = 0;

  // Set to zero to skip waiting for confirmation, set to nonzero to wait and
  // report to DS if action fails.
  int kTimeoutMs = 30;

  // Protobot PID values
  double kF = 0.037;
  double kP = 0.5; 
  double kI = 0;
  double kD = 10;

  double velocityModeUnits;
  private final int RPM_TOLERANCE = 50;
  private final int ENCODER_TICKS_PER_REVOLUTION = 4096;
  private final int ITERATIONS_PER_HUNDRED_MS = 600;

  private TalonShooterSubsystem() {

    // Initialize a private variable for the motor
    if (Config.SHOOTER_MOTOR != -1){
      m_shooter = new WPI_TalonSRX(Config.SHOOTER_MOTOR);
    }

    // Factory Default to prevent unexpected behaviour
    m_shooter.configFactoryDefault();

    // Config sensor used for PID control
    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
    kPIDLoopIDX, kTimeoutMs); 

    // Invert the phase of the sensor so positive output yields positive change
    m_shooter.setSensorPhase(true);
    
		// Config the peak and nominal outputs
		m_shooter.configNominalOutputForward(0, kTimeoutMs);
		m_shooter.configNominalOutputReverse(0, kTimeoutMs);
		m_shooter.configPeakOutputForward(1, kTimeoutMs);
		m_shooter.configPeakOutputReverse(-1, kTimeoutMs);

		// Config the Velocity closed loop gains
		m_shooter.config_kF(kPIDLoopIDX, kF, kTimeoutMs);
		m_shooter.config_kP(kPIDLoopIDX, kP, kTimeoutMs);
		m_shooter.config_kI(kPIDLoopIDX, kI, kTimeoutMs);
    m_shooter.config_kD(kPIDLoopIDX, kD, kTimeoutMs);
  }

  private static class ShooterHolder{
    private static final TalonShooterSubsystem INSTANCE_SHOOTER = new TalonShooterSubsystem();
  }

  /**
   * Returns the singleton instance for the ShooterSubsystem
   */
  public static TalonShooterSubsystem getInstance() {
    return ShooterHolder.INSTANCE_SHOOTER;
  }

  /**
   * Return the motor velocity measured by the encoder
   */
  public double getVelocity(){
    double encoderVelocity = m_shooter.getSelectedSensorVelocity();
    // Add the ability to print to SmartDashboard?
    // Returns change in position per 100ms
    return encoderVelocity;
  }

  /**
   * Set the target RPM to ramp up to. Will eventually get this value 
   * based on distance from power port or other factors.
   */
  public void setRPM(int targetRPM){
    velocityModeUnits = targetRPM * ENCODER_TICKS_PER_REVOLUTION / ITERATIONS_PER_HUNDRED_MS;
  }

  /**
   * Check the actual RPM and compare it with targetRPM
   * to verify that the shooter is up to necessary speed to fire.
   */
  public boolean checkRPM(int targetRPM){
    // Calculate RPM based on the encoder reading
    double calculatedRPM = (m_shooter.getSelectedSensorVelocity() * ITERATIONS_PER_HUNDRED_MS) / ENCODER_TICKS_PER_REVOLUTION;
    // Verify that the motor is running at the target RPM
    return (calculatedRPM < (targetRPM + RPM_TOLERANCE) && calculatedRPM > (targetRPM - RPM_TOLERANCE));
      // I have no idea if +/- 50 tolerance around the RPM is accurate enough
      // Test and change, add setting conditions
  }

  @Override
  public void periodic() {
    if (m_shooter == null) return;
    m_shooter.set(ControlMode.Velocity, velocityModeUnits);
  }
}
