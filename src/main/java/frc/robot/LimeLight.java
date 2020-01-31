package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * LimeLight
 */
class LimeLight {

    public enum CameraMode {
        DRIVE,
        PROCESSING
    }

    private NetworkTable table;

    LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    Vector2d getOffset() {
        return new Vector2d(getDouble("tx"), getDouble("ty"));
    }

    double getDouble(String path) {
        return table.getEntry(path).getDouble(0.0);
    }

    boolean setMode(CameraMode mode) {
        if (mode.equals(CameraMode.DRIVE)) {
            table.getEntry("camMode").setNumber(1.0);
            return true;
        } else {
            table.getEntry("camMode").setNumber(0.0);
            return false;
        }
    }

    public CameraMode getMode() {
        return table.getEntry("camMode").getNumber(0.0).equals(1.0) ? CameraMode.DRIVE : CameraMode.PROCESSING;
    }

}