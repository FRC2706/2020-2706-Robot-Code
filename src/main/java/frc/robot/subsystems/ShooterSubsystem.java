package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.config.Config;

public class ShooterSubsystem extends SubsystemBase {

  // Singleton instance for ShooterSubsystem
  private static final ShooterSubsystem INSTANCE_SHOOTER = new ShooterSubsystem();

  private WPI_TalonSRX m_shooter;

  int kPIDLoopIDX = 0;
  int kTimeoutMs = 30;

  // Protobot PID values
  double kF = 0.037;
  double kP = 0.5; 
  double kI = 0;
  double kD = 10;

  double velocityModeUnits;

  private ShooterSubsystem() {
    // Initialize a private variable for the motor
    if (Config.SHOOTER_MOTOR != -1){
      m_shooter = new WPI_TalonSRX(Config.SHOOTER_MOTOR);
    }

    // Factory Default all hardware to prevent unexpected behaviour
    m_shooter.configFactoryDefault();

    // Config sensor used for PID control
    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIDX, kTimeoutMs); 

    // Description
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

  /**
   * Returns the singleton instance for the ShooterSubsystem
   */
  public static ShooterSubsystem getInstance() {
      return INSTANCE_SHOOTER;
  }

  /**
   * Return the motor velocity measured by the encoder
   */
  public double getVelocity(){
    double encoderVelocity = m_shooter.getSelectedSensorVelocity();
    // print to smartdashboard?
    return encoderVelocity;
  }

  /**
   * Set the shooter motor to a specified RPM using 
   * closed loop velocity control
   */
  public void setRPM(int targetRPM){
    velocityModeUnits = targetRPM * 4096 / 600;
  }

  /**
   * Description
   */
  public void checkRPM(int targetRPM){

    // Calculate RPM based on the encoder reading
    double calculatedRPM = (m_shooter.getSelectedSensorVelocity() * 600) / 4096;

    // Description
    if (calculatedRPM < (targetRPM + 75) && calculatedRPM > (targetRPM - 75)){
      // RPM is accurate enough, set condition so shooter is ready to fire
      // I have no idea if +/- 75 tolerance around the RPM is accurate enough
      // Test and change
    }
  }

  @Override
  public void periodic() {
    if (m_shooter == null) return;
    m_shooter.set(ControlMode.Velocity, velocityModeUnits);
  }
}
