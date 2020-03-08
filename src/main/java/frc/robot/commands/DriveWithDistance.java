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

  //The speed that the robot must move below to move backwards
  private final static double ZERO_SPEED = 0;

  private final static double DEFAULT_RIGHT_SPEED = 0.5;
  private final static double DEFAULT_LEFT_SPEED = 0.5;

  // initialization of the variables
  public double desiredLeftDistance = 0;
  public double desiredRightDistance = 0;
  public double desiredRightEncoderTicks, desiredLeftEncoderTicks;
  public double leftSpeed = 0;
  public double rightSpeed = 0;
  public DistanceType distanceType;
  public double currentConversion;
  public boolean commandFinished = false;
  
  /**
   * Constructor to be used when the input specifications do not provide the right and left speed (assumed to be 0.5)
   * 
   * @param distance The distance to drive
   * @param distanceType The distance unit type to drive in
   */

  public DriveWithDistance(double distance, DistanceType distanceType) {
    this(distance, distanceType, DEFAULT_RIGHT_SPEED, DEFAULT_LEFT_SPEED);
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
    this(distance, distance, distanceType, rightSpeed, leftSpeed);
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
    //Initating drivebase
    this.driveBase = DriveBaseHolder.getInstance();

    //Setting current distance unit
    this.distanceType = distanceType;

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

    //Code for figuring out when the robot is finished driving and handling user input validation

    // Update to the current right/left distance that the robot has driven
    currentRightDistance = DriveBaseHolder.getInstance().getRightDistance();
    currentLeftDistance = DriveBaseHolder.getInstance().getLeftDistance();

    // If the robot has reached or surpassed the desired distance, then stop the
    // robot. Otherwise, keep moving (moving forward OR moving backwards).
    if (Math.abs(currentRightDistance) >= desiredRightDistance) {
      rightSpeed = ZERO_SPEED;
    }

    // If the robot has reached or surpassed the desired distance, then stop the
    // robot. Otherwise, keep moving (moving forward).
    if (Math.abs(currentLeftDistance) >= desiredLeftDistance) {
      leftSpeed = ZERO_SPEED;
    }

    //If the left and right speed are zero, just stop the robot
    if ((leftSpeed == ZERO_SPEED && rightSpeed == ZERO_SPEED) ||
            (desiredLeftDistance >= currentLeftDistance && desiredRightDistance >= currentRightDistance)) {
      commandFinished = true;
    }

    //Taking the changed left and right speed and adding their updated values to the drivebase
    driveBase.tankDrive(leftSpeed, rightSpeed, false);
  }
  
  /**
   * Determines when the command is finished using the current distnce and desired distance
   * 
   */

  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
