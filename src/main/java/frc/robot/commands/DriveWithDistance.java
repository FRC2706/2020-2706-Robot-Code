/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import frc.robot.subsystems.DriveBase.DistanceType;


public class DriveWithDistance extends CommandBase {

//Initializing private version of drivebase object
private final DriveBase driveBase;

//The distance that the robot must move below to move backwards
private final double DISTANCE_NOT_MOVING = 0;

//The speed that the robot must move below to move backwards
private final double SPEED_NOT_MOVING = 0;

private final double DEFAULT_RIGHT_SPEED = 0.5;
private final double DEFAULT_LEFT_SPEED = 0.5;

// initialization of the variables 
  public double currentRightDistance = 0;
  public double currentLeftDistance = 0;
  public double desiredLeftDistance = 0;
  public double desiredRightDistance = 0;
  public double encoderTicks;
  public double desiredRightEncoderTicks, desiredLeftEncoderTicks;
  public double leftSpeed = 0;
  public double rightSpeed = 0;
  public DistanceType distanceType;
  public double currentConversion;
  public boolean rightMotorMovingBackwards = false;
  public boolean leftMotorMovingBackwards = false;
  public boolean robotMovingBackWards = false;
  public boolean isTurning = false;
  public boolean end = false;
  public boolean commandFinished = false;

  
  /**
   * Constructor to be used when the input specifications do not provide the right and left speed (assumed to be 0.5)
   * 
   * @param distance The distance to drive
   * @param distanceType The distance unit type to drive in
   */

  public DriveWithDistance(double distance, DistanceType distanceType) {

    //If you have to move backwards
    if(distance < DISTANCE_NOT_MOVING){
      //Error since you can't move backwards with positive speeds (0.5 is default in this constructor)
      commandFinished = true;
      end = true;

    }
    //Setting current unit type and drive base variables
    this.distanceType = distanceType;
    this.driveBase = DriveBaseHolder.getInstance();
    addRequirements(driveBase);
    assignDistanceType(distanceType);

    //default speed is 0.5 for the left and right talons
    rightSpeed = DEFAULT_RIGHT_SPEED;
    leftSpeed = DEFAULT_LEFT_SPEED;
    this.encoderTicks = distance * this.currentConversion;   
  }


   /**
   * Assigns the current conversion from encoder ticks to distance type
   * 
   * @param distanceType    The distance unit that the robot is using for driving
   */
private void assignDistanceType(DistanceType distanceType) {
	//Setting the current conversion type according to the chosen distance unit
    switch (distanceType) {
      case FEET:
        this.currentConversion = DriveBase.DistanceUnit.FEET.encoderTicks;    
        
        break;
        case METERS:
        this.currentConversion = DriveBase.DistanceUnit.METERS.encoderTicks;    
        
        break;
        case CENTIMETERS:
        this.currentConversion = DriveBase.DistanceUnit.CENTIMETERS.encoderTicks;    
        
        break;
        case INCHES:
        this.currentConversion = DriveBase.DistanceUnit.INCHES.encoderTicks;    
        
        break;

      //Default is meters
      default:

      this.currentConversion = DriveBase.DistanceUnit.METERS.encoderTicks;
      break;

    }
  }

  /**
   * Constructor to be used when the input gives the right and left speed
   * 
   * @param distance     The distance to drive
   * @param distanceType The distance unit type to drive in
   * @param rightSpeed   The right motor speed
   * @param leftSpeed    The left motor speed
   */
  public DriveWithDistance(double distance, DistanceType distanceType, double rightSpeed, double leftSpeed) {

    // Initating drivebase
    this.driveBase = DriveBaseHolder.getInstance();

    // If all the values for distance and speeds are the same magnitude (+/-) then
    // run the robot, otherwise do not.
    if (distance < DISTANCE_NOT_MOVING && rightSpeed < SPEED_NOT_MOVING && leftSpeed < SPEED_NOT_MOVING) {
      robotMovingBackWards = true;
    } else if (distance > DISTANCE_NOT_MOVING && rightSpeed > SPEED_NOT_MOVING && leftSpeed > SPEED_NOT_MOVING) {
      robotMovingBackWards = false;
    }
    // The robot should not move if it has inverse speeds and desired distances
    // since that must be an error
    else {
      driveBase.tankDrive(SPEED_NOT_MOVING, SPEED_NOT_MOVING, false);
      commandFinished = true;
      end = true;
      
    }

    // Setting current distance unit
    this.distanceType = distanceType;

    addRequirements(driveBase);
    
    assignDistanceType(distanceType);

    //Setting the right speed and left speed according to input specifications
    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;

    //Determing the amount of desired encoder ticks to drive by getting the distnace and multiplying by the chosen distance unit
    this.desiredRightEncoderTicks = distance * this.currentConversion;
    this.encoderTicks = distance * this.currentConversion;
  }


   /**
   * Constructor to be used when the input gives the right and left distance
   * 
   * @param rightDistance The distance to drive for right motor
   * @param leftDistance The distance to drive for left motor
   * @param distanceType The distance unit type to drive in
   * @param rightSpeed The right motor speed
   * @param leftSpeed The left motor speed
   */
  public DriveWithDistance(double rightDistance, double leftDistance, DistanceType distanceType, double rightSpeed, double leftSpeed) {
    isTurning = true;

    if(rightDistance < DISTANCE_NOT_MOVING && rightSpeed < DISTANCE_NOT_MOVING){
      rightMotorMovingBackwards = true;
    }
    else if(rightDistance > DISTANCE_NOT_MOVING && rightSpeed > SPEED_NOT_MOVING){
      rightMotorMovingBackwards = false;
    }
    //The robot should not move if it has inverse speeds and desired distances since that must be an error
    else{
      DriveBaseHolder.getInstance().tankDrive(SPEED_NOT_MOVING, SPEED_NOT_MOVING, false);
      commandFinished = true;
      end = true;
    }

    if(leftDistance < SPEED_NOT_MOVING && leftSpeed < SPEED_NOT_MOVING){
      leftMotorMovingBackwards = true;

    }
    else if(leftDistance > DISTANCE_NOT_MOVING && leftSpeed > SPEED_NOT_MOVING){

      leftMotorMovingBackwards = false;
  }
    //The robot should not move if it has inverse speeds and desired distances since that must be an error
    else{
      DriveBaseHolder.getInstance().tankDrive(SPEED_NOT_MOVING, SPEED_NOT_MOVING, false);
      commandFinished = true;
      end = true;
    }

    //Setting current distance unit
    this.distanceType = distanceType;

    //Initating drivebase
    this.driveBase = DriveBaseHolder.getInstance();
    addRequirements(driveBase);
    
    assignDistanceType(distanceType);

    //Setting the right speed and left speed according to input specifications
    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;

    //Determing the amount of desired encoder ticks to drive by getting the distnace and multiplying by the chosen distance unit
    this.desiredRightEncoderTicks = rightDistance * this.currentConversion;   
    this.desiredLeftEncoderTicks = leftDistance * this.currentConversion;

  }
  
  /**
   * Called  one when the command is initially scheduled and determines the desired distance to travel
   * 
   */
  @Override
  public void initialize() {

    //Add the desired amount of encoder ticks to the current distance to get the amount of encoder ticks that the robot needs to drive
    desiredRightDistance = driveBase.getRightDistance() + desiredRightEncoderTicks;
    desiredLeftDistance = driveBase.getLeftDistance() + desiredLeftEncoderTicks;

    addRequirements(DriveBaseHolder.getInstance());

  }

  /**
   * Called every time the scheduler runs while the command is scheduled. (run every 20ms) and determines the current distance
   * 
   */
  @Override
  public void execute() {
   
    //Get the current distance of the right encoder and store value in variable
    double currentRightDistance = DriveBaseHolder.getInstance().getRightDistance();  
    double currentLeftDistance = DriveBaseHolder.getInstance().getLeftDistance();

    //Run motors according to output of the speeds
    driveBase.tankDrive(leftSpeed, rightSpeed, false);

    //Dashboard
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Left Error", desiredLeftDistance - currentLeftDistance);
    SmartDashboard.putNumber("Right Error", desiredRightDistance - currentRightDistance);


    //Code for figuring out when the robot is finished driving and handeling user input validation

    if (!end) {
      // Update to the current right/left distance that the robot has driven
      currentRightDistance = DriveBaseHolder.getInstance().getRightDistance();
      currentLeftDistance = DriveBaseHolder.getInstance().getLeftDistance();

      // The robot is moving forward
      if (!isTurning) {
        // If the robot has reached or surpassed the desired distance, then stop the
        // robot. Otherwise, keep moving (moving forward).
        if (!robotMovingBackWards && currentRightDistance >= desiredRightDistance) {
          driveBase.tankDrive(SPEED_NOT_MOVING, SPEED_NOT_MOVING, false);
          commandFinished = true;

        }
        // If the robot has reached or surpassed the desired distance, then stop the
        // robot. Otherwise, keep moving (moving backward).
        else if (robotMovingBackWards && currentRightDistance <= desiredRightDistance) {
          driveBase.tankDrive(SPEED_NOT_MOVING, SPEED_NOT_MOVING, false);
          commandFinished = true;
        } else {
          commandFinished = false;
        }

      }
      // If the robot is turning, then move the robot for the left side too
      else {

        // If the robot has reached or surpassed the desired distance, then stop the
        // robot. Otherwise, keep moving (moving forward OR moving backwards).
        if ((!rightMotorMovingBackwards && currentRightDistance >= desiredRightDistance) || (rightMotorMovingBackwards && currentRightDistance <= desiredRightDistance)) {
          rightSpeed = SPEED_NOT_MOVING;
        }
        else {
          commandFinished = false;
        }

        // If the robot has reached or surpassed the desired distance, then stop the
        // robot. Otherwise, keep moving (moving forward).
        if ((!leftMotorMovingBackwards && currentLeftDistance >= desiredLeftDistance) || (leftMotorMovingBackwards && currentLeftDistance <= desiredLeftDistance)) {
          leftSpeed = SPEED_NOT_MOVING;
        }
        else {
          commandFinished = false;
        }

        //If the left and right speed are zero, just stop the robot
        if(leftSpeed == SPEED_NOT_MOVING && rightSpeed == SPEED_NOT_MOVING){
          commandFinished = true;
          end = true;
        }

        //If the desired and current distance are equal, don't move the robot
        if(desiredLeftDistance == currentLeftDistance && desiredRightDistance == currentRightDistance)
        {
          commandFinished = true;
          end = true;
        }

        //Taking the changed left and right speed and adding their updated values to the drivebase
        driveBase.tankDrive(leftSpeed, rightSpeed, false);

      }
    }
    
  }
  
  /**
   * Determines when the command is finished using the current distnce and desired distance
   * 
   */

  @Override
  public boolean isFinished() {

    return commandFinished;
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
}
