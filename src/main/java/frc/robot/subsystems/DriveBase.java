/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This allows the robot to drive around
 */

public class DriveBase extends TimedRobot {

  /**
   * Initializing Joystick used for controlling the robot
   */

  private final Joystick m_joystick  = new Joystick(0);

  /**
   *  Initializing 4 motors
   */

      WPI_TalonSRX  leftFrontMotor = new WPI_TalonSRX(1);
      WPI_TalonSRX  leftBackMotor = new WPI_TalonSRX(2); 
      WPI_TalonSRX  rightFrontMotor = new WPI_TalonSRX(3);
      WPI_TalonSRX  rightBackMotor = new WPI_TalonSRX(4); 

      /**
       * Initializing SpeedControllerGroups (left and right)
       */

        SpeedControllerGroup m_left = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
        SpeedControllerGroup m_right= new SpeedControllerGroup(rightFrontMotor, rightBackMotor);

        /**
         * Initliazing DifferentialDrive 
         */

       DifferentialDrive robotDriveBase = new DifferentialDrive(m_left, m_right);

   @Override
   public void robotInit() {
 
   }
 
   @Override
   public void teleopPeriodic() {
 
            /**
             * Using joystick inputs to drive the robot
             */

            robotDriveBase.arcadeDrive(m_joystick.getY(), m_joystick.getX());
     
 
   }

}
