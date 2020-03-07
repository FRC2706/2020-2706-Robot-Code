package frc.robot.commands;

import frc.robot.config.Config;

import frc.robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nettables.VisionCtrlNetTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * VisionAssistedTargetRPM command.
 * NOTE: this command should be run after command TurnToOuterPortCommand
 */
public class VisionAssistedTargetRPM implements Runnable {
  
  //subsystem
    
  //todo: can be configured in config file as well
  private final double SHOOTER_ANGLE_IN_DEGREES  = 46.88;
  private final double TARGET_HEIGHT_IN_METERS = 2.02;
  private final double SHOOTER_WHEEL_RADIUS_IN_CM = 5.5;
  private final double D_CAMERA_SHOOTER_IN_METERS = 0.26;
  private final double ANGLE_CAMERA_SHOOTER = 46.37;

  private final double HALF_OF_GRAVITY = 4.91;
  private final double CONVERSION_NUMBER = 3000;
  private final double METER_PER_FOOT = 0.3048;

  // values from Vision Network Table
  private double distanceToOuterPortInMeters;
  private double yawToOutperPort;

  //adjusted value
  private double adjustedDistanceToOutPortInMeters;
   
  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable;

  //calculated for the shooter
  double targetDistance = 0;
  double targetRPM = 0;


  /**
   * Creates a new VisionAssistedTargetRPM Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAssistedTargetRPM() {

    // Ensure the vision is running in tape mode
    visionControlNetTable.setTapeMode();
    
  }
   
  public void run() {

    yawToOutperPort = visionControlNetTable.yawToOuterPort.get();

    //Read the network table from vision to get the distance from the target.
    distanceToOuterPortInMeters = visionControlNetTable.distanceToOuterPort.get();
    if ( distanceToOuterPortInMeters < 0.0 )
    {
      //Vision can not provide valid detection
      targetRPM = 0;
    }
    else
    {
      //NOTE: unit in the vision network table is feet. Convert it to meters.
      distanceToOuterPortInMeters = distanceToOuterPortInMeters * METER_PER_FOOT ;
  
      //todo: to adjuste the distance for the shooter 
        //adjustedDistanceToOutPortInMeters = ;
      double temp = 2*distanceToOuterPortInMeters * Math.cos(180-ANGLE_CAMERA_SHOOTER);
      double check = distanceToOuterPortInMeters*distanceToOuterPortInMeters 
                    + D_CAMERA_SHOOTER_IN_METERS*D_CAMERA_SHOOTER_IN_METERS - temp;
      if (check<0)
      {
        adjustedDistanceToOutPortInMeters = distanceToOuterPortInMeters;
      }
      else
      {
        adjustedDistanceToOutPortInMeters = Math.sqrt(check);

      }
      
      //Calculate the RPM of the shooter wheel.
      double targetV  = initVelocity( adjustedDistanceToOutPortInMeters);
      targetRPM     = velocityToRPM (targetV);
    }

    // provide feedback to the shuffleboard for Driver Team
    // vision assisted RPM
    SmartDashboard.putNumber("Info: Vision Assisted Target RPM", targetRPM);
  }

 private double initVelocity(double distanceToTargetInMeters) {
    double dTemp;
    //unit: m/s
    double dInitVelocity;

    double dCheck = Math.tan(SHOOTER_ANGLE_IN_DEGREES)*distanceToTargetInMeters - TARGET_HEIGHT_IN_METERS;
    if (dCheck > 0)
    {
        dTemp = Math.sqrt(HALF_OF_GRAVITY/dCheck);
        dInitVelocity = distanceToTargetInMeters/Math.cos(SHOOTER_ANGLE_IN_DEGREES) * dTemp;
    }
    else
    {
        dInitVelocity = 0.0;
        System.out.println("WARNING! Not suitable for shooting!");      
    }
  
    return dInitVelocity;
  
 }

 // convert velocity to RPM
 // velocity: unit m/s
 // return: unit revolutions per minute
private double velocityToRPM( double velocity)
 {     
     double rpm = velocity*CONVERSION_NUMBER/(Math.PI*SHOOTER_WHEEL_RADIUS_IN_CM);
     return rpm;
 }

}