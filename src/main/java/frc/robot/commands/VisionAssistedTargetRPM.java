package frc.robot.commands;

import frc.robot.config.Config;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.nettables.VisionCtrlNetTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * VisionAssistedTargetRPM command.
 * NOTE: this command should be run after command TurnToOuterPortCommand
 */
public class VisionAssistedTargetRPM implements Runnable {
    
 
  // values from Vision Network Table
  private double distanceToOuterPortInMeters;
  
  //adjusted value
  private double adjustedDistanceToOutPortInMeters;
   
  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable = new VisionCtrlNetTable ();

  //calculated RPM
  double targetDistance = 0;
  double targetRPM = 0;

  /**
   * Creates a new VisionAssistedTargetRPM Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAssistedTargetRPM() {

    // Ensure the vision is running in tape mode
    //don't need this any more
    //VisionCtrlNetTable.setTapeMode();    
  }
   
  public void run() {

    //Read the network table from vision 
    distanceToOuterPortInMeters = visionControlNetTable.distanceToOuterPort.get();

    if ( distanceToOuterPortInMeters < 0.0 )
    {
      // Vision can not provide valid detection
      targetRPM = 0;
    }
    else
    {
      // NOTE: unit in the vision network table is feet. Convert it to meters.
      distanceToOuterPortInMeters = distanceToOuterPortInMeters * Config.METER_PER_FOOT ;
  
      // adjuste the distance for the shooter 
      adjustedDistanceToOutPortInMeters = distanceToOuterPortInMeters + Config.D_CAMERA_SHOOTER_IN_METERS;
            
      //Calculate the RPM of the shooter wheel.
      double targetV  = initVelocity( adjustedDistanceToOutPortInMeters);
      targetRPM       = velocityToRPM (targetV);
    }

    // provide feedback to the shuffleboard for Driver Team
    // vision assisted calculated RPM
    SmartDashboard.putNumber("Vision RPM", targetRPM);
  }

 public static double initVelocity(double distanceToTargetInMeters) {
    double dTemp;
    //unit: m/s
    double dInitVelocity;

    double dCheck = Math.tan(Config.SHOOTER_ANGLE_IN_DEGREES)*distanceToTargetInMeters - Config.TARGET_HEIGHT_IN_METERS;
    if (dCheck > 0)
    {
        dTemp = Math.sqrt(Config.HALF_OF_GRAVITY/dCheck);
        dInitVelocity = distanceToTargetInMeters/Math.cos(Config.SHOOTER_ANGLE_IN_DEGREES) * dTemp;
    }
    else
    {
        dInitVelocity = 0.0;    
    }
  
    return dInitVelocity;
  
 }

 // convert velocity to RPM
 // velocity: unit m/s
 // return: unit revolutions per minute
public static double velocityToRPM( double velocity)
 {     
     double rpm = velocity*Config.CONVERSION_NUMBER/(Math.PI*Config.SHOOTER_WHEEL_RADIUS_IN_CM);
     return rpm;
 }

}