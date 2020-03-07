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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.config.Config;
import frc.robot.sensors.AnalogSelector;
import frc.robot.subsystems.*;
import frc.robot.commands.ArcadeDriveWithJoystick;
import frc.robot.commands.DriveWithDistance;
import frc.robot.commands.DriveWithTime;

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
    
  // RobotContainer is a singleton class
  private static RobotContainer currentInstance;

  // The robot's subsystems and commands are defined here...    
  private Joystick driverStick;
  private Joystick controlStick;
  private AnalogSelector analogSelectorOne;
  private AnalogSelector analogSelectorTwo;
  private Command driveCommand;
  private Command intakeCommand;
  private Command reverseFeeder;
  private Command moveArmToSetpoint;
  private Command reverseArmManually;
  private Command positionPowercell;
  private Command rampShooterCommand;
  private Command incrementFeeder;
  private Command perfectPosition;
  private Logger logger = Logger.getLogger("RobotContainer");
  private Command runFeeder;

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
      
        // Instantiate the intake command and bind it
        intakeCommand = new OperatorIntakeCommand();
        new JoystickButton(controlStick, XboxController.Button.kBumperLeft.value).whenHeld(intakeCommand);

        reverseFeeder = new ReverseFeeder();
        new JoystickButton(controlStick, XboxController.Button.kB.value).whenHeld(reverseFeeder);

        runFeeder = new RunFeederCommand(-0.2);
        new JoystickButton(controlStick, XboxController.Button.kY.value).whenHeld(runFeeder);

        incrementFeeder = new IncrementFeeder(-FeederSubsystem.FEEDERSUBSYSTEM_INCREMENT_TICKS.get());
        new JoystickButton(controlStick, XboxController.Button.kX.value).whenHeld(incrementFeeder);

        rampShooterCommand = new SpinUpShooter(1500);
        new JoystickButton(controlStick, XboxController.Button.kA.value).toggleWhenActive(rampShooterCommand);

        driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y, Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS, true);
        DriveBaseHolder.getInstance().setDefaultCommand(driveCommand);

        positionPowercell = new PositionPowercellCommand();
        new JoystickButton(controlStick, XboxController.Button.kBumperRight.value).toggleWhenActive(positionPowercell, true);


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        //Robot has only one selector
        int selectorOne = 0;
        if (analogSelectorOne != null)
            selectorOne = analogSelectorOne.getIndex();
        logger.info("Selectors: " + selectorOne);

        if (selectorOne == 0) {
            // This is our 'do nothing' selector
            return null;
        }

        else if (selectorOne == 1) {
         /*
          * When the selector is set to one, the robot will run for x seconds at y left motor speed and z right motor speed
          */

              // for the inputed variables: DriveWithTime(seconds (double), left motors speed (double), right motors speed (double))
              return new DriveWithTime(Config.AUTO_DRIVE_TIME, Config.AUTO_LEFT_MOTOR_SPEED, Config.AUTO_RIGHT_MOTOR_SPEED);
         }

        else if(selectorOne == 2){

            //List distance, then the drive unit (in option of meters, cm, inches or feet), and then the right and left speed (if not specified, it is 0.5)
            return new DriveWithDistance(Config.AUTO_DISTANCE, Config.DEFAULT_UNIT, Config.AUTO_RIGHT_MOTOR_SPEED, Config.AUTO_LEFT_MOTOR_SPEED);
        }
      
        // Also return null (do nothing) if this ever gets to here because safety
        return null;
    }

    public void joystickRumble(double leftValue, double rightValue) {
        //Joystick rumble (driver feedback). leftValue/rightValue sets vibration force.
        driverStick.setRumble(RumbleType.kLeftRumble, leftValue);
        driverStick.setRumble(RumbleType.kRightRumble, rightValue);
    }

    /**
     * Initialize the current RobotContainer instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new RobotContainer();
        }
    }

    public static RobotContainer getInstance() {
        init();
        return currentInstance;
    }
    
    
}