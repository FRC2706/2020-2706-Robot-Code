package frc.robot.commands;

import frc.robot.config.Config;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nettables.VisionCtrlNetTable;

/**
 * VisionAssistedShooter command.
 * NOTE: this command should be run after command TurnToOuterPortCommand
 */
public class VisionAssistedShooter extends CommandBase {
  
  //subsystem
  private final ShooterSubsystem shooter;
    
  //todo: can be configured in config file as well
  //todo: measure the radius for the shooting wheel
  private final double SHOOTER_ANGLE_IN_DEGREES  = 60.0;
  private final double TARGET_HEIGHT_IN_METERS = 2.49;
  private final double SHOOTER_WHEEL_RADIUS_IN_CM = 10;
  private final double HALF_OF_GRAVITY = 4.91;
  private final double CONVERSION_NUMBER = 3000;

  // values from Vision Network Table
  private double distanceToOuterPort;
   
  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable;

  //calculated for the shooter
  double targetDistance = 0;
  double targetRPM = 0;


  /**
   * Creates a new VisionAssistedShooter Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAssistedShooter(ShooterSubsystem subsystem) {
   
    //subsystem
    shooter = subsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

    // Ensure the vision is running in tape mode
    visionControlNetTable.setTapeMode();

    //Read the network table from vision to get the distance from the power port.
    //distance from vision
    //NOTE: unit should be meter. If not, need conversion here.
    distanceToOuterPort = visionControlNetTable.distanceToOuterPort.get();

    //todo: to adjuste the distance for the shooter 
    //targetDistance

    //Calculate the RPM of the shooter wheel.
    double targetV  = initVelocity( distanceToOuterPort);
    targetRPM     = velocityToRPM (targetV);
    
  }

  @Override
  public void execute() {

    //Set the shooter to the target RPM.
    shooter.setTargetRPM((int) targetRPM);

    //todo: provide feedback to the shuffleboard for Driver Team
    
  }

  @Override
  public boolean isFinished() {
      // This command should only be run once
      return true;
  }

  @Override
    public void end(boolean interrupted) {
        
    }


  double initVelocity(double distanceToTargetInMeters) {
    double dCheck = Math.tan(SHOOTER_ANGLE_IN_DEGREES)*distanceToTargetInMeters - TARGET_HEIGHT_IN_METERS;
    double dTemp;

    //unit: m/s
    double dInitVelocity;
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
double velocityToRPM( double velocity)
 {     
     double rpm = velocity*CONVERSION_NUMBER/(Math.PI*SHOOTER_WHEEL_RADIUS_IN_CM);
     return rpm;
 }

}