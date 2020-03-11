/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;
import java.util.logging.Logger;
import java.lang.Math;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

public class DrivetrainPIDTurnDelta extends CommandBase {
    // Declare PD variables
    private Supplier<Double> pGain = Config.DRIVETRAIN_P;
    private Supplier<Double> dGain = Config.DRIVETRAIN_D;

    // Get the drivebase and pigeon
    private final DriveBase drivebase;
    private PigeonIMU pigeonIMU;

    boolean isDone = false;

    // The delta of which you want to turn in degrees
    private double deltaDegree;

    // Target angle in degrees
    private double targetAngle;

    // Current angle in degrees
    private double currentAngle;

    // Initiate forwardSpeed
    private double forwardSpeed;

    // Acceptable error in the angle
    private double acceptableError;

    // The maximum time the command is allowed to run, set to null for no time limit
    private Double maxTime;

    private Logger logger = Logger.getLogger("DTPIDDelta");

    // A timer to ensure the command doesn't get stuck and the robot cannot drive
    private Timer timer;


    /**
     * Allows the robot to turn and move forward or back by itself
     * @param deltaDegree The degree you want the robot to turn, negative is left, positive is right
     * @param forwardSpeed The speed you want the robot to move, (-1 to 1) negative being backwards and positive being forwards
     */
    public DrivetrainPIDTurnDelta(double deltaDegree, double forwardSpeed, double acceptableError, Double maxTime) {
        //Get the supplied values
        this.deltaDegree = deltaDegree;
        this.forwardSpeed = forwardSpeed;
        this.acceptableError = acceptableError;
        this.maxTime = maxTime;

        //Set the drivebase
        this.drivebase = DriveBaseHolder.getInstance();
        addRequirements(this.drivebase);
        pigeonIMU = drivebase.getPigeon();
        currentAngle = drivebase.getCurrentAngle();
        logger.addHandler(Config.logFileHandler);

        // Instantiate the timer
        if(this.maxTime != null) {
            timer = new Timer();
        }



    }

    @Override
    public boolean isFinished() {
        return pigeonIMU == null || isDone || (maxTime != null && timer.get() >= maxTime);
    }

    @Override
    public void initialize() {

        //Get the target angle
        targetAngle = drivebase.getCurrentAngle() + deltaDegree;

        isDone = false;

        if(maxTime != null) {
            timer.start();
        }

    }

    @Override
    public void execute() {
        if(pigeonIMU != null) {
            // Set starting throttle
            double turnThrottle = 0;

            // Get current angular rate
            double[] xyz_dps = new double[3];
            pigeonIMU.getRawGyro(xyz_dps);
            //Get z axis angular rate
            double currentAngularRate = xyz_dps[2];

            //Get current angle
            currentAngle = drivebase.getCurrentAngle();

            if (Math.abs(targetAngle - currentAngle) < acceptableError) {
                isDone = true;
            }

            //Do PD
            turnThrottle = (targetAngle - currentAngle) * pGain.get() - (currentAngularRate) * dGain.get();

            //Run motors according to the output of PD
            drivebase.tankDrive(-turnThrottle + forwardSpeed, turnThrottle + forwardSpeed, false);
        } else {
            DriverStation.reportError("Pigeon IMU is null", false);
        }
    }
}
