
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;

/**
 * Class to allow for usage of WPI and Comand functions with TankDrive
 */
public class TankDrive extends CommandBase {

  private final Supplier<Double> leftSpeed;
  private final Supplier<Double> rightSpeed;

  private final boolean squareInputs;
  private final boolean initBrake;

  /**
   * 
   * @param leftSpeed The values to use for the left side
   * @param rightSpeed The values to use for the right side
   * @param squareInputs Whether or not to square the forward and rotation values
   * @param initBrake Whetehr or not to start and end the command in brake or coast mode
   */
  protected TankDrive(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed, boolean squareInputs, boolean initBrake) {
    //Ensure that this command is the only one to run on the drive base
    //Requires must be included to use this command as a default command for the drive base
    addRequirements(DriveBase.getInstance());

    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    this.squareInputs = squareInputs;
    this.initBrake = initBrake;

  }


  @Override
  public void initialize() {
    //Prepare for driving by human
    DriveBase.getInstance().setOpenLoopVoltage();
    DriveBase.getInstance().setBrakeMode(initBrake);
  }

 
  @Override
  public void execute() {
    //Pass values to drive base to make the robot move
    DriveBase.getInstance().tankDrive(leftSpeed.get(),rightSpeed.get(),squareInputs);
  }


  @Override
  public void end(boolean interrupted) {
    //go back to disabled mode
    DriveBase.getInstance().setDisabledMode();

  
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
