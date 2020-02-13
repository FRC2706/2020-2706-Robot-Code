package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

public class Agitator extends SubsystemBase {

    private VictorSPX agitatorMotor;


public Agitator (VictorSPX agitatorMotor)
{//Defining the agitator motor
    this.agitatorMotor = agitatorMotor;
    } 
        
public void runAgitator()
{// Runs the agitator at full speed
    agitatorMotor.set(AGITATOR_SPEED);
    } 

public void stopAgitator ()
{ // Stops the agitator
    agitatorMotor.set(0.0);
    }

public void reverseAgitator ()
{
    agitatorMotor.set(AGITATOR_REVERSE_SPEED);
    } //Runs the agitator in reverse (Might be useless?)


//Variables:
private final double AGITATOR_SPEED = 1.0;
private final double AGITATOR_REVERSE_SPEED = -1.0;
}

/* Code will be bulletproofed later by getting readings from sensors
and config entry before running*/