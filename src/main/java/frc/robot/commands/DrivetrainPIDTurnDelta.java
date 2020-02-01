/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class DrivetrainPIDTurnDelta extends CommandBase {
    //Delcare PD variables
    private Supplier<Double> pGain = DriveBase.DRIVETRAIN_P;
    private Supplier<Double> dGain = DriveBase.DRIVETRAIN_D;

    //get the drivebase and pigeon
    private final DriveBase drivebase;
    private PigeonIMU _pidgey;

    //The delta of which you want to turn in degrees
    private double deltaDegree;

    //Target angle in degrees
    private double targetAngle;

    //Current angle in degrees
    private double currentAngle;

    //Initiate forwardSpeed
    private double forwardSpeed;

    /**
     * Allows the robot to turn and move forward or back by itself
     * @param deltaDegree The degree you want the robot to turn, negative is left, positive is right
     * @param forwardSpeed The speed you want the robot to move, (-1 to 1) negative being backwards and positive being forwards
     */
    public DrivetrainPIDTurnDelta(double deltaDegree, double forwardSpeed) {
        //Get the supplied delta
        this.deltaDegree = deltaDegree;

        //Get supplied forwardspeed
        this.forwardSpeed = forwardSpeed;

        //Set the drivebase
        addRequirements(DriveBase.getInstance());
        this.drivebase = DriveBase.getInstance();
        _pidgey = drivebase.getPigeon();
        double currentAngle = drivebase.getCurrentAngle();

    }

    @Override
    public void initialize() {
        //Get the target angle
        targetAngle = drivebase.getCurrentAngle() + deltaDegree;
    }

    @Override
    public void execute() {
        //Set starting throttle
        double turnThrottle = 0;

        //Get current angular rate
        double[] xyz_dps = new double[3];
        _pidgey.getRawGyro(xyz_dps);
        //Get z axis angular rate
        double currentAngularRate = xyz_dps[2];

        //Get current angle
        currentAngle = drivebase.getCurrentAngle();

        //Do PD
        turnThrottle = (targetAngle - currentAngle) * pGain.get() - (currentAngularRate) * dGain.get();

        //Run motors according to the output of PD
        drivebase.tankDrive(-turnThrottle + forwardSpeed, turnThrottle + forwardSpeed, false);
    }
}
