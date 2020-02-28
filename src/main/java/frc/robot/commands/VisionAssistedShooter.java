package frc.robot.commands;


//import frc.robot.subsystems;
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
  private final double theta  = 60.0;    //unit: degree
  private final double height = 2.49;    //unit: meter 
  private final double radius = 10;//unit cm

  //values from Vision Network Table
  private double distanceToOuterPort;
   
  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable;

  //calculated for the shooter
  double targetDistance = 0;
  double targetRPM = 0;


  /**
   * Creates a new ExampleCommand.
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
    double targetV  = initVelocity( distanceToOuterPort, height, theta);
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

    //d: unit meter
    //h: unit meter
    //theta: unit degree
    //return: unit m/s
  double initVelocity(double d, double h, double theta) {
    double dCheck = Math.tan(theta)*d - h;
    double dTemp;

    //unit: m/s
    double dInitVelocity;
    if (dCheck > 0)
    {
         dTemp = Math.sqrt(4.91/dCheck);
         dInitVelocity = d/Math.cos(theta) * dTemp;
    }
    else
    {
         dInitVelocity = 0.0;
         System.out.println("WARNING! Not suitable for shooting!");      
     }

    return dInitVelocity;
  
 }

 // convert velocity t0 RPM
 // velocity: unit m/s
 // return: unit revolutions per minute
double velocityToRPM( double velocity)
 {     
     double rpm = velocity*3000/(Math.PI*radius);
     return rpm;
 }

}