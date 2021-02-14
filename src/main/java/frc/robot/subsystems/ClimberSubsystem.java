/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class ClimberSubsystem extends SubsystemBase {

    private WPI_TalonSRX climberTalon;

    /**
     * Creates a new ClimberSubsystem.
     */
    public ClimberSubsystem() {
        if(Config.CLIMBER_TALON != -1) initializeClimber();
    }

    private void initializeClimber(){
        climberTalon = new WPI_TalonSRX(Config.CLIMBER_TALON);

        climberTalon.configPeakOutputForward(1);
        climberTalon.configPeakOutputReverse(0);
    }

    public boolean isActive() {
        return climberTalon != null;
    }

    private static class ClimberHolder {
        private static final ClimberSubsystem INSTANCE_CLIMBER = new ClimberSubsystem();
    }

    public static ClimberSubsystem getInstance() {
        return ClimberHolder.INSTANCE_CLIMBER;
    }

    public void runClimber(double climbSpeed){
        climberTalon.set(climbSpeed);
    }

    public void stopClimber(){
        climberTalon.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
