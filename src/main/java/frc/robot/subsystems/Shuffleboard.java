/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class Shuffleboard extends SubsystemBase {
  private final Joystick joystick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private int SecondsLeftInMatch = 150; // 150 (seconds) = 2:30 minutes

  private static Shuffleboard INSTANCE;

  /**
   * Creates a new Shuffleboard.
   */
  private Shuffleboard() {
    INSTANCE = new Shuffleboard();
    CameraServer.getInstance().startAutomaticCapture();
    m_timer.start();
    SmartDashboard.putNumber("Time Left", SecondsLeftInMatch);
  }

  public static Shuffleboard getINSTANCE() {
    return INSTANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // shift alt F
    if (SecondsLeftInMatch > 0) {
      SmartDashboard.putNumber("Timer", m_timer.get());
      if (m_timer.get() > 1) {
        m_timer.stop();
        m_timer.reset();
        SmartDashboard.putNumber("Time Left", SecondsLeftInMatch);
        SecondsLeftInMatch = SecondsLeftInMatch - 1;
        m_timer.start();
      }
      SmartDashboard.putBoolean("ENDGAME", SecondsLeftInMatch < 30);
      SmartDashboard.putBoolean("GAME OVER", SecondsLeftInMatch < 1);

      boolean buttonValueA; // A is 1
      boolean buttonValueB; // B is 2
      boolean buttonValueX; // X is 3
      boolean buttonValueY; // Y is 4

      double LeftXvalue = joystick.getRawAxis(0);
      double LeftYvalue = joystick.getRawAxis(1);
      double RightXvalue = joystick.getRawAxis(4);
      double RightYvalue = joystick.getRawAxis(5);

      // Left Joystick
      SmartDashboard.putNumber("Left Joystick X Value", LeftXvalue);
      SmartDashboard.putNumber("Left Joystick Y Value", LeftYvalue * -1);
      SmartDashboard.putBoolean("Left X Too Slow?", LeftXvalue < -0.9);
      SmartDashboard.putBoolean("Left X Too Fast?", LeftXvalue > 0.9);
      SmartDashboard.putBoolean("Left Y Too Fast?", LeftYvalue < -0.9);
      SmartDashboard.putBoolean("Left Y Too Slow?", LeftYvalue > 0.9);

      // Right Joystick
      SmartDashboard.putNumber("Right Joystick X Value", RightXvalue);
      SmartDashboard.putNumber("Right Joystick Y Value", RightYvalue * -1);
      SmartDashboard.putBoolean("Right X Too Slow?", RightXvalue < -0.9);
      SmartDashboard.putBoolean("Right X Too Fast?", RightXvalue > 0.9);
      SmartDashboard.putBoolean("Right Y Too Fast?", RightYvalue < -0.9);
      SmartDashboard.putBoolean("Right Y Too Slow?", RightYvalue > 0.9);

      if (LeftXvalue < -0.9 || LeftXvalue > 0.9 || LeftYvalue < -0.9 || LeftYvalue > 0.9 || RightXvalue < -0.9 || RightXvalue > 0.9
          || RightYvalue < -0.9 || RightYvalue > 0.9) {
        joystick.setRumble(RumbleType.kLeftRumble, 1);
        joystick.setRumble(RumbleType.kRightRumble, 1);
      } else {
        if (LeftXvalue < -0.8 || LeftXvalue > 0.8 || LeftYvalue < -0.8 || LeftYvalue > 0.8 || RightXvalue < -0.8 || RightXvalue > 0.8
            || RightYvalue < -0.8 || RightYvalue > 0.8) {
          joystick.setRumble(RumbleType.kLeftRumble, 0.5);
          joystick.setRumble(RumbleType.kRightRumble, 0.5);
        } else {
          joystick.setRumble(RumbleType.kLeftRumble, 0);
          joystick.setRumble(RumbleType.kRightRumble, 0);
        }
      }

      buttonValueA = joystick.getRawButton(1); // Button A
      SmartDashboard.putBoolean("A", buttonValueA);
      buttonValueB = joystick.getRawButton(2); // Button B
      SmartDashboard.putBoolean("B", buttonValueB);
      buttonValueX = joystick.getRawButton(3); // Button X
      SmartDashboard.putBoolean("X", buttonValueX);
      buttonValueY = joystick.getRawButton(4); // Button Y
      SmartDashboard.putBoolean("Y", buttonValueY);

      SmartDashboard.putNumber("Auto Selector", RobotContainer.getInstance().analogSelectorOne.getIndex());

    } else {
      // game over signal can be added here
    }
  }
}
