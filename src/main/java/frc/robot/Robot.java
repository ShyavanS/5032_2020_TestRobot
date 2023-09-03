package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.Vector2d;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.LimeLight;
import frc.robot.vision.LimeLightTarget;
import frc.robot.vision.UltrasonicSensor;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {
  public enum DriveMode {
    DRIFT, STALL;
  }

  private DifferentialDrive robot;
  private Joystick joystick;
  private LimeLight limelight;

  private UltrasonicSensor sensor;
  public static Encoder encoder;
  
  //Colour sensor initialization and set colour values
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private Color colourMatch;
  private boolean scanningColour = false;

  // Solenoid brake pneumatics
  private DoubleSolenoid brake;

  private CameraSwitcher cameraSwitcher;

  private double previousSpeed;
  private double previousRotation;
  private DriveMode driveMode = DriveMode.STALL;

  private WPI_TalonFX falcon500;

  private WPI_VictorSPX victor1;
  private WPI_VictorSPX victor2;
  private WPI_VictorSPX colourSpinner;

  private WPI_TalonSRX driveLeftTop;
  private WPI_TalonSRX driveLeftBottom;
  private WPI_TalonSRX driveRightTop;
  private WPI_TalonSRX driveRightBottom;  

  private boolean pressed;

  @Override
  public void robotInit() {
    // Drive
    driveLeftTop = new WPI_TalonSRX(0);
    driveLeftBottom = new WPI_TalonSRX(1);
    driveRightTop = new WPI_TalonSRX(2);
    driveRightBottom = new WPI_TalonSRX(3);

    // Speed Controllers
    SpeedControllerGroup speedControllerLeft = new SpeedControllerGroup(driveLeftTop, driveLeftBottom); // 4 and 3 should be swapped back for drive to work
    SpeedControllerGroup speedControllerRight = new SpeedControllerGroup(driveRightTop, driveRightBottom);
    //speedControllerRight.setInverted(true);

    victor1 = new WPI_VictorSPX(2);
    victor2 = new WPI_VictorSPX(3);
    colourSpinner = new WPI_VictorSPX(4);

    // Set up solenoid
    brake = new DoubleSolenoid(0, 1);
    brake.set(DoubleSolenoid.Value.kReverse);
    pressed = true;

    // Drive Train
    robot = new DifferentialDrive(speedControllerLeft, speedControllerRight);
    robot.setSafetyEnabled(false);
    robot.setDeadband(0);

    falcon500 = new WPI_TalonFX(0);

    // Inputs
    joystick = new Joystick(0);
    sensor = new UltrasonicSensor(0);

    // Vision
    limelight = new LimeLight();
    cameraSwitcher = new CameraSwitcher("Vision", joystick, 1, 0, 1);

    falcon500.setSelectedSensorPosition(0);
    falcon500.getSensorCollection().setIntegratedSensorPosition(0, 1);

    // Encoder
    encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    encoder.setMaxPeriod(.1);
    encoder.setMinRate(10);
    encoder.setDistancePerPulse(5);
    encoder.setReverseDirection(true);
    encoder.setSamplesToAverage(7);
    encoder.reset();

    // Colour sensor setup
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void robotPeriodic() {
    int count = encoder.get();
    double distance = encoder.getRaw();
    double distance2 = encoder.getDistance();
    double rate = encoder.getRate();
    boolean direction = encoder.getDirection();
    boolean stopped = encoder.getStopped();
    System.out.println("Encoder count: " + count);
    System.out.println("Encoder distance: " + distance);
    System.out.println("Encoder rate: " + rate);
    System.out.println("Encoder direction: " + direction);
    System.out.println("Encoder Stopped?: " + stopped);
  }

  @Override
  public void teleopPeriodic() {
    // Main drive
    double sensitivity = calculateSensitivity();
    

    double rotation = joystick.getZ() * (sensitivity * 0.8);
    double speed = -joystick.getY() * sensitivity;

    if (driveMode == DriveMode.DRIFT) {
      rotation += previousRotation * 3;
      rotation /= 4;
      speed += previousSpeed * 3;
      speed /= 4;
    }

    robot.arcadeDrive(rotation, speed);

    previousRotation = rotation;
    previousSpeed = speed;

    // Utility
    cameraSwitcher.tick();

    // Drive Mode
    if (joystick.getRawButtonPressed(8)) {
      if (driveMode == DriveMode.DRIFT) driveMode = DriveMode.STALL;
      else driveMode = DriveMode.DRIFT;
    }

    if (joystick.getRawButton(9)) {
      falcon500.set(calculateSensitivity());
    } else {
      falcon500.set(0.0);
    }

    if (joystick.getRawButton(5) && pressed == true) {
      brake.set(DoubleSolenoid.Value.kForward);
      pressed = false;
    } else if (joystick.getRawButton(5) && pressed == false) {
      brake.set(DoubleSolenoid.Value.kReverse);
      pressed = true;
    }

    if (joystick.getRawButton(3)) { // Button 3 - Victor 1
      victor1.set(calculateSensitivity()); // Set speed of this motor to sensitivity of slider
      // victor2.set(-calculateSensitivity() / 20); // Other motor spins slightly to aid the first motor
    } else if (!joystick.getRawButton(4)) {
      victor1.set(0.0); //Stop this motor when the other button is pressed
    } else if (joystick.getRawButtonReleased(3)) {
      victor2.set(calculateSensitivity()/10); // Spin other motor for a bit as soon as the button is released to relieve slack
    }

    if (joystick.getRawButton(4)) { // Button 4 - Victor 2
      victor2.set(calculateSensitivity());
      // victor1.set(-calculateSensitivity() / 20); 
    } else if (!joystick.getRawButton(3)) {
      victor2.set(0.0);
    } else if (joystick.getRawButtonReleased(4)) {
      victor1.set(calculateSensitivity()/10);
    }

    // Limelight

    if (joystick.getRawButtonPressed(3)) {
      LimeLight.CameraMode cameraMode = limelight.getMode();
      if (cameraMode.equals(LimeLight.CameraMode.DRIVE)) {
        limelight.setMode(LimeLight.CameraMode.PROCESSING);
      } else {
        limelight.setMode(LimeLight.CameraMode.DRIVE);
      }
    }

    if (joystick.getRawButtonPressed(2)) limelight.startTarget(LimeLight.Pipeline.REFLECTIVE_TAPE);
    if (joystick.getRawButton(2)) {
      LimeLightTarget target = limelight.getTarget();
      if (target == null) return;

      Vector2d offset = target.getOffset();
      if (offset.x == 0) return;

      double slowingRadius = 16;
      double maxVelocity = 0.485;

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

      robot.tankDrive(desired, desired);
    }
    if (joystick.getRawButtonReleased(2)) limelight.finishTarget();
    updateSmartboard();

    // Colour Sensor match
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {
    switch (gameData.charAt(0)) {
      case 'B' :
        colourMatch = kBlueTarget;
        break;
      case 'G' :
        colourMatch = kGreenTarget;
        break;
      case 'R' :
        colourMatch = kRedTarget;
        break;
      case 'Y' :
        colourMatch = kYellowTarget;
        break;
      default :
        break;
      }
    } else {}

    if (joystick.getRawButton(7) || scanningColour == true) {
      if (match.color != colourMatch) {
        colourSpinner.set(0.25);
        scanningColour = true;
      } else {
        colourSpinner.set(0.0);
        scanningColour = false;
      }
    }
  }

  private void updateSmartboard() {
    SmartDashboard.putNumber("Sensor (vlts)", sensor.getVoltage());
    SmartDashboard.putNumber("Sensor (dst)", sensor.getDistance());
    SmartDashboard.putNumber("sel pos", falcon500.getSelectedSensorPosition()/2048); // 2048 raw sensor units is equal to 1 revollution of the motor shaft
    SmartDashboard.putNumber("sel vel", falcon500.getSelectedSensorVelocity());
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