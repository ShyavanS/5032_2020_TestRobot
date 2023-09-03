package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

import java.nio.channels.Pipe;
import java.util.Vector;

/**
 * LimeLight
 */
@Getter
public class LimeLight {

    @Getter
    @AllArgsConstructor
    public enum CameraMode {
        DRIVE(1),
        PROCESSING(0);

        private int value;

        static CameraMode fromValue(int value) {
            for (CameraMode mode: CameraMode.values()) {
                if (mode.getValue() == value) return mode;
            }
            return null;
        }
    }

    @AllArgsConstructor
    @Getter
    public enum Pipeline {
        REFLECTIVE_TAPE(0);

        private int value;

        static Pipeline fromValue(int value) {
            for (Pipeline mode: Pipeline.values()) {
                if (mode.getValue() == value) return mode;
            }
            return null;
        }
    }

    @AllArgsConstructor
    @Getter
    public enum LEDMode {
        CURRENT(0), OFF(1), BLINK(2), ON(3);

        private int value;

        static LEDMode fromValue(int value) {
            for (LEDMode mode: LEDMode.values()) {
                if (mode.getValue() == value) return mode;
            }
            return null;
        }
    }

    private NetworkTable table;
    private boolean targetting = false;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void startTarget(Pipeline pipeline) {
        setPipeline(pipeline);
        setLEDMode(LEDMode.ON);

        targetting = true;
    }

    public LimeLightTarget getTarget() {
        if (hasTarget() && isTargetting()) {
            setLEDMode(LEDMode.ON);
            double tx = getDouble("tx");
            double ty = getDouble("ty");
            double ta = getDouble("ta");
            double ts = getDouble("ts");

            double thor = getDouble("thor");
            double tvert = getDouble("tvert");

            return new LimeLightTarget(thor, tvert, ts, ta, new Vector2d(tx, ty));
        } else if (isTargetting()) {
            setLEDMode(LEDMode.BLINK);
        }
        return null;
    }

    public void finishTarget() {
        if (isTargetting()) {
            setLEDMode(LEDMode.OFF);
        }
    }

    public void setLEDMode(LEDMode mode) {
        table.getEntry("ledMode").setNumber(mode.getValue());
    }

    public LEDMode getLEDMode() {
        return LEDMode.fromValue(table.getEntry("ledMode").getNumber(0.0).intValue());
    }

    public void setPipeline(Pipeline pipeline) {
        table.getEntry("pipeline").setNumber(pipeline.getValue());
    }

    public Pipeline getPipeline() {
        return Pipeline.fromValue(table.getEntry("getpipe").getNumber(0.0).intValue());
    }

    public boolean hasTarget() {
        return getDouble("tv") == 1.0;
    }

    public void setMode(CameraMode mode) {
        table.getEntry("camMode").setNumber(mode.getValue());
    }

    public CameraMode getMode() {
        return CameraMode.fromValue(table.getEntry("camMode").getNumber(0.0).intValue());
    }

    private double getDouble(String path) {
        return table.getEntry(path).getDouble(0.0);
    }

}