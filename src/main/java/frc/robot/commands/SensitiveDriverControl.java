/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        addRequirements(DriveBase.getInstance());
        this.driveBase = DriveBase.getInstance();
        this.j = j;
        
    }

    /**
     * Drives with sensitive control
     */
    @Override
    public void execute() {
        this.driveBase.sensitiveSteering = true;
        this.driveBase.tankDrive(j.getRawAxis(4), j.getRawAxis(4), false);
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