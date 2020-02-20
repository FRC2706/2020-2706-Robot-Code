
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

/**
 * Class to allow for usage of WPI and Comand functions with TankDrive
 */
public class TankDrive extends CommandBase {

  private final Supplier<Double> leftSpeed;
  private final Supplier<Double> rightSpeed;

  private final boolean squareInputs;
  private final boolean initBrake;

  private final DriveBase driveBase;

  /**
   * 
   * @param leftSpeed The values to use for the left side
   * @param rightSpeed The values to use for the right side
   * @param squareInputs Whether or not to square the forward and rotation values
   * @param initBrake Whether or not to start and end the command in brake or coast mode
   */
  protected TankDrive(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed, boolean squareInputs, boolean initBrake) {
    //Ensure that this command is the only one to run on the drive base
    //Requires must be included to use this command as a default command for the drive base
    this.driveBase = DriveBaseHolder.getInstance();
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    this.squareInputs = squareInputs;
    this.initBrake = initBrake;
    addRequirements(this.driveBase);
  }


  @Override
  public void initialize() {
    //Prepare for driving by human
    this.driveBase.setDriveMode(DriveBase.DriveMode.OpenLoopVoltage);
    this.driveBase.setNeutralMode(initBrake ? NeutralMode.Brake : NeutralMode.Coast);
  }

 
  @Override
  public void execute() {
    //Pass values to drive base to make the robot move
    this.driveBase.tankDrive(leftSpeed.get(),rightSpeed.get(),squareInputs);
  }


  @Override
  public void end(boolean interrupted) {
    //go back to disabled mode
    this.driveBase.setNeutralMode(NeutralMode.Brake);

  
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
