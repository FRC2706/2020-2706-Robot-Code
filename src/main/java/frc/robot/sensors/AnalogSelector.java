package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class AnalogSelector implements Sendable {

    private static final double MODE_ZERO_LOW_VOLTAGE = 0;
    private static final double MODE_ZERO_HIGH_VOLTAGE = 2.4;

    private static final double MODE_ONE_LOW_VOLTAGE = 2.4;
    private static final double MODE_ONE_HIGH_VOLTAGE = 2.8;

    private static final double MODE_TWO_LOW_VOLTAGE = 2.8;
    private static final double MODE_TWO_HIGH_VOLTAGE = 3.1;

    private static final double MODE_THREE_LOW_VOLTAGE = 3.1;
    private static final double MODE_THREE_HIGH_VOLTAGE = 3.4;

    private static final double MODE_FOUR_LOW_VOLTAGE = 3.4;
    private static final double MODE_FOUR_HIGH_VOLTAGE = 3.7;

    private static final double MODE_FIVE_LOW_VOLTAGE = 3.7;
    private static final double MODE_FIVE_HIGH_VOLTAGE = 3.9;

    private static final double MODE_SIX_LOW_VOLTAGE = 3.9;
    private static final double MODE_SIX_HIGH_VOLTAGE = 4.05;

    private static final double MODE_SEVEN_LOW_VOLTAGE = 4.05;
    private static final double MODE_SEVEN_HIGH_VOLTAGE = 4.15;

    private static final double MODE_EIGHT_LOW_VOLTAGE = 4.15;
    private static final double MODE_EIGHT_HIGH_VOLTAGE = 4.24;
    
    private static final double MODE_NINE_LOW_VOLTAGE = 4.24;
    private static final double MODE_NINE_HIGH_VOLTAGE = 4.34;

    private static final double MODE_TEN_LOW_VOLTAGE = 4.34;
    private static final double MODE_TEN_HIGH_VOLTAGE = 4.4;

    private static final double MODE_ELEVEN_LOW_VOLTAGE = 4.4;
    private static final double MODE_ELEVEN_HIGH_VOLTAGE = 4.49;

    private static final double MODE_TWELVE_LOW_VOLTAGE = 4.49;
    private static final double MODE_TWELVE_HIGH_VOLTAGE = 5;

    private static final Range[] VOLTAGE_RANGES = {
        new Range(MODE_ZERO_LOW_VOLTAGE, MODE_ZERO_HIGH_VOLTAGE),
        new Range(MODE_ONE_LOW_VOLTAGE, MODE_ONE_HIGH_VOLTAGE), 
        new Range(MODE_TWO_LOW_VOLTAGE, MODE_TWO_HIGH_VOLTAGE),
        new Range(MODE_THREE_LOW_VOLTAGE, MODE_THREE_HIGH_VOLTAGE), 
        new Range(MODE_FOUR_LOW_VOLTAGE, MODE_FOUR_HIGH_VOLTAGE), 
        new Range(MODE_FIVE_LOW_VOLTAGE, MODE_FIVE_HIGH_VOLTAGE),
        new Range(MODE_SIX_LOW_VOLTAGE, MODE_SIX_HIGH_VOLTAGE), 
        new Range(MODE_SEVEN_LOW_VOLTAGE, MODE_SEVEN_HIGH_VOLTAGE), 
        new Range(MODE_EIGHT_LOW_VOLTAGE, MODE_EIGHT_HIGH_VOLTAGE),
        new Range(MODE_NINE_LOW_VOLTAGE, MODE_NINE_HIGH_VOLTAGE), 
        new Range(MODE_TEN_LOW_VOLTAGE, MODE_TEN_HIGH_VOLTAGE), 
        new Range(MODE_ELEVEN_LOW_VOLTAGE, MODE_ELEVEN_HIGH_VOLTAGE),
        new Range(MODE_TWELVE_LOW_VOLTAGE, MODE_TWELVE_HIGH_VOLTAGE),
    };
    private final AnalogInput analogInput;

    public AnalogSelector(int channel) {
        this.analogInput = new AnalogInput(channel);
        SendableRegistry.addChild(this, analogInput);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Voltage", analogInput::getAverageVoltage, null);
        builder.addDoubleProperty("Index", this::getIndex, null);
    }

    public int getIndex() {

        final double voltage = analogInput.getAverageVoltage();

        int index = 0;
        // Check each voltage range
        for (int i = 0; i < VOLTAGE_RANGES.length; i++) {
            // Check if the voltage is within the current voltage range
            if (VOLTAGE_RANGES[i].isWithin(voltage)) {
                index = i;
                break;
            }
        }

        return index - 1;
    }

    public double getVoltage() {
        double voltage = analogInput.getVoltage();
        return voltage;
    }

    public double getAverageVoltage() {
        double avgVoltage = analogInput.getAverageVoltage();
        return avgVoltage;
    }

    public static class Range {
        public final double min, max;

        public Range(double min, double max) {
            this.min = min;
            this.max = max;
        }

        /**
         * Determines if the number is within this range.
         *
         * @param number The number to be tested.
         * @return True if it's within range, false otherwise.
         */
        public boolean isWithin(final double number) {
            return min <= number && number < max;
        }
    }
}