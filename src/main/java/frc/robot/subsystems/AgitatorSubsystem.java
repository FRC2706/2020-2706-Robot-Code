package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class AgitatorSubsystem extends SubsystemBase {

    //Variables:
    private final double AGITATOR_SPEED = 0.75;
    private VictorSPX agitatorMotor;

    public static AgitatorSubsystem INSTANCE = new AgitatorSubsystem();

    private AgitatorSubsystem() {
        //Defining the agitator motor
        agitatorMotor = new VictorSPX(Config.AGITATOR_MOTOR);
    }

    public void runAgitator() {
        // Runs the agitator at full speed
        agitatorMotor.set(ControlMode.PercentOutput, AGITATOR_SPEED);
    }

    public void stopAgitator() {
        // Stops the agitator
        agitatorMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void reverseAgitator() {
        //Runs the agitator in reverse
        agitatorMotor.set(ControlMode.PercentOutput, -AGITATOR_SPEED);
    }
}