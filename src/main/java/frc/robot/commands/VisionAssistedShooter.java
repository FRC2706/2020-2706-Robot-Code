package frc.robot.commands;

import frc.robot.config.Config;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nettables.VisionCtrlNetTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * VisionAssistedShooter command.
 * NOTE: this command should be run after command TurnToOuterPortCommand
 */
public class VisionAssistedShooter extends CommandBase {
  
  //subsystem
  private final ShooterSubsystem shooter;

  // values from Vision Network Table
  private double distanceToOuterPortInMeters;

  //adjusted value
  private double adjustedDistanceToOutPortInMeters;
   
  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable = new VisionCtrlNetTable ();

  //calculated for the shooter
  double targetDistance = 0;
  double targetRPM = 0;


  /**
   * Creates a new VisionAssistedShooter Command.
   *
   */
  public VisionAssistedShooter( ) {
   
    //subsystem
    shooter = ShooterSubsystem.getInstance();
   
    if (shooter.isActive()){
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(shooter);
    }
  }

  @Override
  public void initialize() {

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
      distanceToOuterPortInMeters = distanceToOuterPortInMeters * Config.METER_PER_FOOT ;

      // adjuste the distance for the shooter 
      adjustedDistanceToOutPortInMeters = distanceToOuterPortInMeters + Config.D_CAMERA_SHOOTER_IN_METERS;

      //Calculate the RPM of the shooter wheel.
      double targetV  = VisionAssistedTargetRPM.initVelocity( adjustedDistanceToOutPortInMeters);
      targetRPM       = VisionAssistedTargetRPM .velocityToRPM (targetV);
    }
  }

  @Override
  public void execute() {

    //Set the shooter to the target RPM.
    shooter.setTargetRPM((int) targetRPM);

    // provide feedback to the shuffleboard for Driver Team
    // vision assisted RPM
    SmartDashboard.putNumber("Vision Assisted Shooter RPM", targetRPM);
  }

  @Override
  public boolean isFinished() {
      // This command should only be run once
      return true;
  }

  @Override
    public void end(boolean interrupted) {
        
    }

  
}