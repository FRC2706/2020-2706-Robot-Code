/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ShooterSubsystem extends SubsystemBase {
  

  private static final ShooterSubsystem INSTANCE_SHOOTER = new ShooterSubsystem();

  private WPI_TalonSRX m_shooter = new WPI_TalonSRX(deviceNumber);

  int kPIDLoopIDX = 0;
  int kTimeoutMs = 30;

  double kF = 0;
  double kP = 0;
  double kI = 0;
  double kD = 0;

  private ShooterSubsystem() {
    

    m_shooter.configFactoryDefault();    

    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIDX, kTimeoutMs); 

    m_shooter.setSensorPhase(true);
    
		/* Config the peak and nominal outputs */
		m_shooter.configNominalOutputForward(0, kTimeoutMs);
		m_shooter.configNominalOutputReverse(0, kTimeoutMs);
		m_shooter.configPeakOutputForward(1, kTimeoutMs);
		m_shooter.configPeakOutputReverse(-1, kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		m_shooter.config_kF(kPIDLoopIDX, kF, kTimeoutMs);
		m_shooter.config_kP(kPIDLoopIDX, kP, kTimeoutMs);
		m_shooter.config_kI(kPIDLoopIDX, kI, kTimeoutMs);
    m_shooter.config_kD(kPIDLoopIDX, kD, kTimeoutMs);
  }

  public void getVelocity(){
    SmartDashboard.putNumber("Shooter Velocity", m_shooter.getSelectedSensorVelocity());
  }

  public void rampRPM(int targetRPM){
    double velocityModeUnits = targetRPM * 4096 / 600;

    m_shooter.set(ControlMode.Velocity, velocityModeUnits);
  }

  @Override
  public void periodic() {
    
  }
}
