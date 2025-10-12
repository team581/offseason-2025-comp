package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.ArmConfig;
import frc.robot.config.RobotConfig.ClawConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.DeployConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.SingulatorConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.generated.RobotTunerConstants;

class CompConfig {
  private static final String CANIVORE_NAME = RobotTunerConstants.kCANBus.getName();
  private static final String RIO_CAN_NAME = "rio";

  public static final RobotConfig competitionBot =
      new RobotConfig(
          new IntakeConfig(
              CANIVORE_NAME,
              25,
              new Debouncer(0.3, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new ClawConfig(
              CANIVORE_NAME, 99, 99, false, new Debouncer(0), new TalonFXConfiguration()),
          new DeployConfig(
              CANIVORE_NAME,
              20,
              99,
              148,
              1.75,
              1.0,
              20.0,
              1,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(48))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new ClimberConfig(
              CANIVORE_NAME,
              21,
              22,
              23,
              24,
              0.0,
              0.0,
              // Climb motor
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(NeutralModeValue.Brake)
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withPeakReverseDutyCycle(0))
                  .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(75.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(100)
                          .withSupplyCurrentLimit(100)),
              // Cancoder
              new CANcoderConfiguration()
                  .withMagnetSensor(
                      new MagnetSensorConfigs()
                          .withMagnetOffset(-0.44921875)
                          .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                          .withAbsoluteSensorDiscontinuityPoint(0.5)),
              // Grab motor
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(35)
                          .withSupplyCurrentLimit(35))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)),
              new CANrangeConfiguration()
                  .withProximityParams(
                      new ProximityParamsConfigs()
                          .withProximityThreshold(0.05)
                          .withProximityHysteresis(0.01)
                          .withMinSignalStrengthForValidMeasurement(7000))),
          new SingulatorConfig(
              CANIVORE_NAME,
              28,
              29,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast)),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          // TODO: add radius and sensor-mechanism ratio
          new ElevatorConfig(
              CANIVORE_NAME,
              999,
              0.0,
              0.0,
              0.0,
              9999.0,
              0.0,
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2 * Math.PI))),
          new ArmConfig(
              RIO_CAN_NAME,
              999,
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(NeutralModeValue.Brake)
                          .withInverted(InvertedValue.CounterClockwise_Positive))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKV(0.0)
                          .withKD(0.0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Arm_Cosine))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(0.0)
                          .withMotionMagicCruiseVelocity(0.0)
                          .withMotionMagicExpo_kA(0.0)
                          .withMotionMagicExpo_kV(0.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((64.0 / 8.0) * (90.0 / 10.0)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(60.0)
                          .withStatorCurrentLimit(60.0)),
              0.0,
              0.0),
          new VisionConfig(
              0.005,
              0.8,
              // Translation: Positive X = Forward, Positive Y = Left, Positive Z = Up
              // Rotation: Positive X = Roll Right, Positive Y = Pitch Down, Positive Z = Yaw Left

              // Robot pose to calibration rig
              new Pose3d(
                  0.0,
                  Units.inchesToMeters(0.0),
                  Units.inchesToMeters(0.0),
                  new Rotation3d(0.0, 0.0, 0.0)),

              // Left Back Limelight
              // Forward: -0.2845308, Right: -0.2695448, Up: 0.2321306, Roll: 0.0, Pitch: 5.0, Yaw:
              // 50.0
              new Pose3d(
                  Units.inchesToMeters(-11.202),
                  Units.inchesToMeters(10.612),
                  Units.inchesToMeters(9.139),
                  new Rotation3d(
                      Units.degreesToRadians(0.0),
                      Units.degreesToRadians(-5.0),
                      Units.degreesToRadians(50.0))),

              // Left Front Limelight
              // Forward: 0.2472182, Right: -0.3260344, Up: 0.2320544, Roll: 0.0, Pitch: 5.0, Yaw:
              // 130.0
              new Pose3d(
                  Units.inchesToMeters(9.733),
                  Units.inchesToMeters(12.836),
                  Units.inchesToMeters(9.136),
                  new Rotation3d(
                      Units.degreesToRadians(0.0),
                      Units.degreesToRadians(-5.0),
                      Units.degreesToRadians(130.0))),

              // Right Limelight
              // Forward: 0.187706, Right: 0.186182, Up: 0.2042922, Roll: 0.0, Pitch: 10.0, Yaw:
              // -90.0
              new Pose3d(
                  Units.inchesToMeters(7.5),
                  Units.inchesToMeters(-7.0),
                  Units.inchesToMeters(9.5),
                  new Rotation3d(
                      Units.degreesToRadians(0.0),
                      Units.degreesToRadians(-10.0),
                      Units.degreesToRadians(-87.0))),
              new Pose3d(
                  Units.inchesToMeters(-8.0),
                  Units.inchesToMeters(10.78),
                  Units.inchesToMeters(32.75),
                  new Rotation3d(
                      Units.degreesToRadians(-5.0),
                      Units.degreesToRadians(28.0),
                      Units.degreesToRadians(-10.0)))),
          new SwerveConfig(new PhoenixPIDController(5.75, 0, 0), true, true, true));

  private CompConfig() {}
}
