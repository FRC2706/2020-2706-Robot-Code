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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DrivetrainPIDTurnDelta;
import frc.robot.commands.OperatorIntakeCommand;
import frc.robot.config.Config;
import frc.robot.config.XboxValue;
import frc.robot.sensors.AnalogSelector;
import frc.robot.subsystems.DriveBase;
import frc.robot.commands.ArcadeDriveWithJoystick;
import frc.robot.commands.CollectCenterBalls;
import frc.robot.commands.DriveWithDistance;
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
  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    // The robot's subsystems and commands are defined here...
    
    private Joystick driverStick;
    private Joystick controlStick;
    public static AnalogSelector analogSelectorOne;
    private AnalogSelector analogSelectorTwo;
    private Command driveCommand;
    private Command intakeCommand;
    private Command sensitiveDriverControlCommand;
    private Logger logger = Logger.getLogger("RobotContainer");

    private final double RIGHT_SPEED = 0.2;
    private final double LEFT_SPEED = 0.2;
    private final double DISTANCE = 1;
    private final DriveBase.DistanceType DEFAULT_UNIT= DriveBase.DistanceType.METERS;

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
       // intakeCommand = new OperatorIntakeCommand();
        
        /**
         * Select drive mode for robot
         */       
        driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y, Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS);
        DriveBase.getInstance().setDefaultCommand(driveCommand);

        sensitiveDriverControlCommand = new SensitiveDriverControl(driverStick);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        int selectorOne = 0, selectorTwo = 0;
        if (analogSelectorOne != null)
            selectorOne = analogSelectorOne.getIndex();
        if (analogSelectorTwo != null)
            selectorTwo = analogSelectorTwo.getIndex();
        logger.info("Selectors: " + selectorOne + " " + selectorTwo);

        if (selectorOne == 0 && selectorTwo == 0) {
            // This is our 'do nothing' selector
            return null;
        }

         else if (selectorOne == 1) {
         /*
          * When the selector is set to one, the robot will run for x seconds at y left motor speed and z right motor speed
          */

              // for the inputed variables: DriveWithTime(seconds (double), left motors speed (double), right motors speed (double))
              return new DriveWithTime(1.0, 0.2, 0.2);
         }

        else if(selectorOne == 2){

            //List distance, then the drive unit (in option of meters, cm, inches or feet), and then the right and left speed (if not specified, it is 0.5)
            return new DriveWithDistance(1, 1, DEFAULT_UNIT, 0.2, 0.2);

        }
        else if(selectorOne == 3){

            return new CollectCenterBalls();


        }
        else if(selectorOne == 4){

            return new DriveWithDistance(1.5, DEFAULT_UNIT, 0.2, 0.2);
        }
        // Also return null if this ever gets to here because safety
        return null;
    }
    
    
}


