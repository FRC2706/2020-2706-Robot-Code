/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.OperatorIntakeCommand;
import frc.robot.commands.SpinUpShooter;
import frc.robot.config.Config;
import frc.robot.config.XboxValue;
import frc.robot.sensors.AnalogSelector;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveBase;
import frc.robot.commands.ArcadeDriveWithJoystick;
import frc.robot.commands.DriveWithTime;
import frc.robot.commands.SensitiveDriverControl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.logging.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  // The robot's subsystems and commands are defined here...    
  private Joystick driverStick;
  private Joystick controlStick;
  private AnalogSelector analogSelectorOne;
  private AnalogSelector analogSelectorTwo;
  private Command driveCommand;
  private Command intakeCommand;
  private Command emptyFeederCommand;
    private Command reverseFeeder;
  private Command moveArm;
  private Command rampShooterCommand;
  private Command incrementFeeder;
  private Logger logger = Logger.getLogger("RobotContainer");
  private final double AUTO_DRIVE_TIME = 1.0;
  private final double AUTO_LEFT_MOTOR_SPEED = 0.2;
  private final double AUTO_RIGHT_MOTOR_SPEED = 0.2;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        logger.addHandler(Config.logFileHandler);
        if (Config.ANALOG_SELECTOR_ONE != -1) {
            analogSelectorOne = new AnalogSelector(Config.ANALOG_SELECTOR_ONE);
        }
        if (Config.ANALOG_SELECTOR_TWO != -1) {
            analogSelectorTwo = new AnalogSelector(Config.ANALOG_SELECTOR_TWO);
        }

        ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverStick = new Joystick(0);
        controlStick = new Joystick(1);

        /**
         * Select drive mode for robot
         */
      
        // Instantiate the intake command and bind it
        intakeCommand = new OperatorIntakeCommand();
        new JoystickButton(controlStick, XboxController.Button.kBumperLeft.value).whenHeld(intakeCommand);

        emptyFeederCommand = new EmptyFeeder();
        new JoystickButton(controlStick, XboxController.Button.kB.value).whenHeld(emptyFeederCommand);

        reverseFeeder = new ReverseFeeder();
        new JoystickButton(controlStick, XboxController.Button.kX.value).whenHeld(reverseFeeder);

        // Instantiate the shooter ramping command and bind it
//        rampShooterCommand = new SpinUpShooter();
//        new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(rampShooterCommand);

        driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y, Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS, true);
        DriveBase.getInstance().setDefaultCommand(driveCommand);

        rampShooterCommand = new SpinUpShooter();
        new JoystickButton(controlStick, XboxController.Button.kA.value).whenHeld(rampShooterCommand);

//        moveArm = new MoveArmManuallyCommand();
//        new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(moveArm);

//        incrementFeeder = new IncrementFeeder();
//        new JoystickButton(driverStick, XboxController.Button.kY.value).whenPressed(incrementFeeder, false);



//        sensitiveDriverControlCommand = new SensitiveDriverControl(driverStick);
//
//        JoystickButton turnToYaw = new JoystickButton(driverStick, XboxValue.XBOX_A_BUTTON.getPort());
//        turnToYaw.whenPressed(new TurnToOuterPortCommand(true, Config.maxYawErrorOuterPortCommand.get(), Config.maxTimeOuterPortCommand.get()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        int selectorOne = 1, selectorTwo = 1;
        if (analogSelectorOne != null){
            selectorOne = analogSelectorOne.getIndex();
        }
        if (analogSelectorTwo != null){
            selectorTwo = analogSelectorTwo.getIndex();
        }
        logger.info("Selectors: " + selectorOne + " " + selectorTwo);

        if (selectorOne == 0 && selectorTwo == 0) {
            // This is our 'do nothing' selector
            return null;
        }

        else if (selectorOne == 1 || selectorTwo == 1) {

            return new DriveWithTime(AUTO_DRIVE_TIME,  AUTO_LEFT_MOTOR_SPEED,  AUTO_RIGHT_MOTOR_SPEED);
        }
        // Also return null if this ever gets to here because safety
        return null;
    }
    
}