package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends CommandBase {
  
  private ShooterSubsystem shooterSubsystem;

  // Fluidconstant?
  int RPM = 2000;
  boolean doneRamping;

  public SpinUpShooter() {
    shooterSubsystem = ShooterSubsystem.getInstance();
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Hardcoded RPM for testing
    shooterSubsystem.setRPM(RPM);
    shooterSubsystem.periodic();
    shooterSubsystem.checkRPM(RPM);
    doneRamping = shooterSubsystem.checkRPM(RPM);
    if(doneRamping == true){
      // Print to console
      System.out.println("calculatedRPM is within 50 units of targetRPM");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
