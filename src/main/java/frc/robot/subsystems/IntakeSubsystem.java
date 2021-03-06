package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

public class IntakeSubsystem extends ConditionalSubsystemBase {
    /**
     * The Singleton instance of this IntakeSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    // The supplier of the intake speed
    private final static FluidConstant<Double> INTAKE_SPEED = new FluidConstant<>("intake-target-speed", 0.80d)
            .registerToTable(Config.constantsTable);

    // The intake motor (if any)
    private TalonSRX intakeMotor;

    /**
     * Creates a new instance of this IntakeSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private IntakeSubsystem() {

        createCondition("operatorActivated", SubsystemConditionStates.TELEOP);
        createCondition("dummyPlaceholder", SubsystemConditionStates.AUTO).setState(false);
        if (Config.INTAKE_MOTOR != -1) {
            intakeMotor = new TalonSRX(Config.INTAKE_MOTOR);
        }

        intakeMotor.setInverted(true);

    }

    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method
     * should be used -- {@code IntakeSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        // The intakeMotor will be null if the Config entry for it was -1. (Meaning this robot doesn't have an intake)
        if (intakeMotor == null) return;

        // If all the conditions are met, set the motor to run at the target speed, otherwise stop.
        if (checkConditions()) {
            intakeMotor.set(ControlMode.PercentOutput, INTAKE_SPEED.get());
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0d);
        }
    }
}

//hi
