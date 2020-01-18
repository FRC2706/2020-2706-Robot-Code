/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.config.Config;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveBase extends SubsystemBase {

   private static DriveBase currentInstance;

     private static void init(){

      if(currentInstance == null){
         currentInstance = new DriveBase();

      }

     } 
     public static DriveBase getInstance(){
        init();
        return currentInstance;
     }

      private static DifferentialDrive robotDriveBase;
      
   // private static DriveBase getInstance(){

   // }

   private WPI_TalonSRX leftFrontTalon, leftRearTalon, rightFrontTalon, rightRearTalon;

   private SpeedControllerGroup left, right;
   private DriveBase(){

      leftFrontTalon = new WPI_TalonSRX(Config.LEFT_FRONT_TALON);
      leftRearTalon = new WPI_TalonSRX(Config.LEFT_REAR_TALON);
      rightFrontTalon = new WPI_TalonSRX(Config.RIGHT_FRONT_TALON);
      rightRearTalon = new WPI_TalonSRX(Config.RIGHT_REAR_TALON);

      left = new SpeedControllerGroup(leftFrontTalon, leftRearTalon);
      right = new SpeedControllerGroup(rightFrontTalon, rightRearTalon);

      robotDriveBase = new DifferentialDrive(left, right);

   }

   public void arcadeDrive(double forwardVal, double rotateVal, boolean squareInputs){
      
      robotDriveBase.arcadeDrive(forwardVal,rotateVal,squareInputs);


   }

   public void stop(){
      left.stopMotor();
      right.stopMotor();
   }
   
   private void selectEncoderStandard(){

      leftFrontTalon.config

   }

}
