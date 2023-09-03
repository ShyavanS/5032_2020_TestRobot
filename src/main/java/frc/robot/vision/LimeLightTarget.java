package frc.robot.vision;

import edu.wpi.first.wpilibj.drive.Vector2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter @AllArgsConstructor
public class LimeLightTarget {

    private double width;
    private double height;
    private double rotation;
    private double area;

    private Vector2d offset;

}
