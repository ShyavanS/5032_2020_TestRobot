package frc.robot.vision;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor {

    private static final double VOLTS_TO_DISTANCE = 1.0;

    private AnalogInput input;

    public UltrasonicSensor(int channel) {
        input = new AnalogInput(channel);
    }

    public double getDistance() {
        return getVoltage() * UltrasonicSensor.VOLTS_TO_DISTANCE;
    }

    public double getVoltage() {
        return input.getVoltage();
    }

}
