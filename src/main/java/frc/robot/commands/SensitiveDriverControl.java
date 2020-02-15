/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.config.Config;
import frc.robot.subsystems.*;

/**
 * Triggers a modifier which limits the speed the robot can turn and only allows it to turn in place
 */
public class SensitiveDriverControl extends CommandBase {
    
    //get the drivebase
    private DriveBase driveBase;
    private Joystick j;

    public SensitiveDriverControl(Joystick j) {
        //set the drivebase
        this.driveBase = DriveBase.getInstance();
        addRequirements(this.driveBase);
        this.j = j;

        //Separation of concern- no other class uses this code
        new JoystickButton(j, XboxController.Button.kBumperLeft.value).whenHeld(this);
        
    }

    /**
     * Drives with sensitive control
     */
    @Override
    public void execute() {
        this.driveBase.sensitiveSteering = true;
        this.driveBase.tankDrive(j.getRawAxis(Config.RIGHT_CONTROL_STICK_X), j.getRawAxis(Config.RIGHT_CONTROL_STICK_X), false);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    /**
     * Stops the drivebase
     */
    @Override
    public void end(boolean interrupted) {
        this.driveBase.sensitiveSteering = false;
        this.driveBase.stop();
    }
}