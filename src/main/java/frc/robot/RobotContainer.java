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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDriveWithJoystick;
import frc.robot.config.Config;
import frc.robot.sensors.AnalogSelector;
import frc.robot.subsystems.DriveBase;

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
    
    private Joystick driverStick, controlStick;
    private AnalogSelector analogSelectorOne, analogSelectorTwo;
    private Command driveCommand, emptyFeederCommand, incrementFeederCommand, intakeCommand;
    private Logger logger = Logger.getLogger("RobotContainer");
    
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
        
        /**
         * Select drive mode for robot
         */
        
        driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y, Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS);
        DriveBase.getInstance().setDefaultCommand(driveCommand);
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        int selectorOne = 0, selectorTwo = 0;
        if (analogSelectorOne != null) selectorOne = analogSelectorOne.getIndex();
        if (analogSelectorTwo != null) selectorTwo = analogSelectorTwo.getIndex();
        logger.info("Selectors: " + selectorOne + " " + selectorTwo);
        
        if (selectorOne == 0 && selectorTwo == 0) {
            // This is our 'do nothing' selector
            return null;
        }
        
        // If we had more auto options I'd add them here lol
        
        // Also return null if this ever gets to here because safety
        return null;
    }
}


