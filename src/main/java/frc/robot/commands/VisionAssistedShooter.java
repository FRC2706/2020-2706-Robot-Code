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
  private final ShooterSubsystem m_shooter;
    
  //todo: can be configured in config file as well
  private final double m_theta  = 60.0;    //unit: degree
  private final double m_height = 2.49;    //unit: meter

  //values from Vision Network Table
  private double m_distanceToOuterPort;
   
  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable;

  //calculated for the shooter
  double m_targetRPM = 0;
  double m_targetDistance = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAssistedShooter(ShooterSubsystem subsystem, DriveBase driveBase) {
   
    //subsystem
    m_shooter = subsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
    // Ensure the vision is running in tape mode
    visionControlNetTable.setTapeMode();

    //Read the network table from vision to get the distance from the power port.
    //distance from vision
    m_distanceToOuterPort = visionControlNetTable.distanceToOuterPort.get();

    //todo: to adjuste the distance for the shooter 
    //m_targetDistance

    //Calculate the RPM of the shooter wheel.
    double targetV  = initVelocity( m_distanceToOuterPort, m_height, m_theta);
    m_targetRPM     = velocityToRPM (targetV);
    
  }

  @Override
  public void execute() {

    //Set the shooter to the target RPM.
    m_shooter.setTargetRPM((int) m_targetRPM);

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

  double initVelocity(double d, double h, double theta) {
    double dCheck = Math.tan(theta)*d - h;
    double dTemp;
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

 //todo: convert velocity t0 RPM
double velocityToRPM( double velocity)
 {
     //todo: set to the constant table
     double radius = 10;//cm
     //then velocity's unit should be cm/minute

     double rpm = velocity/(2*Math.PI*radius);
     return rpm;
 }

}