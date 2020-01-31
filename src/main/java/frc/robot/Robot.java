package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {
  private DifferentialDrive robot;
  private Joystick joystick;
  private WPI_VictorSPX shot1;
  private WPI_VictorSPX shot2;
  private LimeLight limelight;

  private ADIS16448_IMU adis;
  private PIDController controller;
  private CameraSwitcher cameraSwitcher;


  @Override
  public void robotInit() {
    SpeedControllerGroup speedControllerLeft = createVictorController(2, 3);
    SpeedControllerGroup speedControllerRight = createVictorController(1, 4);

    // SHOT
    shot1 = new WPI_VictorSPX(5);
    shot2 = new WPI_VictorSPX(6);

    shot1.setInverted(true);

    speedControllerRight.setInverted(true);
    robot = new DifferentialDrive(speedControllerLeft, speedControllerRight);

    joystick = new Joystick(0);

    adis = new ADIS16448_IMU();
    controller = new PIDController(1, 1, 1);

    limelight = new LimeLight();
    cameraSwitcher = new CameraSwitcher("Vision", joystick, 1, 0, 1);

    SmartDashboard.putData("PID", controller);
  }

  @Override
  public void teleopPeriodic() {
    double sensitivity = calculateSensitivity();

    double rotation = joystick.getZ() * (sensitivity);
    double speed = -joystick.getY() * sensitivity;

    robot.arcadeDrive(rotation, speed);

    if (joystick.getRawButton(11)) {
      shot1.set(-1);
      shot2.set(-1);
    } else {
      shot1.set(0);
      shot2.set(0);
    }

    cameraSwitcher.tick();

    if (joystick.getRawButtonPressed(3)) {
      LimeLight.CameraMode cameraMode = limelight.getMode();
      if (cameraMode.equals(LimeLight.CameraMode.DRIVE)) {
        limelight.setMode(LimeLight.CameraMode.PROCESSING);
      } else {
        limelight.setMode(LimeLight.CameraMode.DRIVE);
      }
    }

    if (joystick.getRawButton(2)) {
      Vector2d offset = limelight.getOffset();
      if (offset.x == 0) return;

      double slowingRadius = 25;
      double maxVelocity = 0.7;

      double xOffset = offset.x;
      double xTarget = 0.0;
      double desired = xTarget - xOffset;
      double distance = Math.abs(desired);

      desired = desired > 0 ? -1 : 1;
      if (distance < slowingRadius) {
        desired *= maxVelocity * (distance / slowingRadius);
      } else {
        desired *= maxVelocity;
      }

      if (distance > 1) {
        desired = desired > 0 ?
                  Math.max(desired, 0.4) :
                  Math.min(desired, -0.4);
      } else {
        robot.stopMotor();
        return;
      }

      SmartDashboard.putNumber("Desired Velocity", desired);

      robot.arcadeDrive(desired, 0);
    }
  }

  private double calculateSensitivity() {
    double sensitivity = joystick.getRawAxis(3);
    sensitivity += 1;
    sensitivity /= 2;
    sensitivity = Math.abs(sensitivity - 1);
    return sensitivity;
  }

  private SpeedControllerGroup createVictorController(int victor, int... others) {
    List<WPI_VictorSPX> victors = Arrays.stream(others).mapToObj(WPI_VictorSPX::new).collect(Collectors.toList());

    return new SpeedControllerGroup(new WPI_VictorSPX(victor), victors.toArray(new WPI_VictorSPX[] {}));
  }

}
