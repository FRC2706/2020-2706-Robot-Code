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

public class Shuffleboard extends SubsystemBase {
  private final Joystick joystick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private int SecondsLeft = 150;

  private static Shuffleboard INSTANCE = new Shuffleboard();

  /**
   * Creates a new Shuffleboard.
   */
  private Shuffleboard() {
    CameraServer.getInstance().startAutomaticCapture();
    m_timer.start();
    SmartDashboard.putNumber("Time Left", SecondsLeft);
  }

  public static Shuffleboard getINSTANCE() {
    return INSTANCE;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // shift alt F
    if (SecondsLeft > 0) {
      SmartDashboard.putNumber("Timer", m_timer.get());
      if (m_timer.get() > 1) {
        m_timer.stop();
        m_timer.reset();
        SmartDashboard.putNumber("Time Left", SecondsLeft);
        SecondsLeft = SecondsLeft - 1;
        m_timer.start();
      }
      SmartDashboard.putBoolean("ENDGAME", SecondsLeft < 30);
      SmartDashboard.putBoolean("GAME OVER", SecondsLeft < 1);

      boolean buttonValueA; // A is 1
      boolean buttonValueB; // B is 2
      boolean buttonValueX; // X is 3
      boolean buttonValueY; // Y is 4

      double XvalueL = joystick.getRawAxis(0);
      double YvalueL = joystick.getRawAxis(1);
      double XvalueR = joystick.getRawAxis(4);
      double YvalueR = joystick.getRawAxis(5);

      // Left Joystick
      SmartDashboard.putNumber("Joystick XL Value", XvalueL);
      SmartDashboard.putNumber("Joystick YL Value", YvalueL * -1);
      SmartDashboard.putBoolean("XL Too Slow?", XvalueL < -0.9);
      SmartDashboard.putBoolean("XL Too Fast?", XvalueL > 0.9);
      SmartDashboard.putBoolean("YL Too Fast?", YvalueL < -0.9);
      SmartDashboard.putBoolean("YL Too Slow?", YvalueL > 0.9);

      // Right Joystick
      SmartDashboard.putNumber("Joystick XR Value", XvalueR);
      SmartDashboard.putNumber("Joystick YR Value", YvalueR * -1);
      SmartDashboard.putBoolean("XR Too Slow?", XvalueR < -0.9);
      SmartDashboard.putBoolean("XR Too Fast?", XvalueR > 0.9);
      SmartDashboard.putBoolean("YR Too Fast?", YvalueR < -0.9);
      SmartDashboard.putBoolean("YR Too Slow?", YvalueR > 0.9);

      if (XvalueL < -0.9 || XvalueL > 0.9 || YvalueL < -0.9 || YvalueL > 0.9 || XvalueR < -0.9 || XvalueR > 0.9
          || YvalueR < -0.9 || YvalueR > 0.9) {
        joystick.setRumble(RumbleType.kLeftRumble, 1);
        joystick.setRumble(RumbleType.kRightRumble, 1);
      } else {
        if (XvalueL < -0.8 || XvalueL > 0.8 || YvalueL < -0.8 || YvalueL > 0.8 || XvalueR < -0.8 || XvalueR > 0.8
            || YvalueR < -0.8 || YvalueR > 0.8) {
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

    } else {
      // game over signal can be added here
    }
  }
}
