package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.config.RobotConfig.ArmConfig;
import frc.robot.config.RobotConfig.ClawConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.DeployConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.SingulatorConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.vision.limelight.LimelightModel;

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
              // TODO: BRINGUP: CHECK SENSOR FLIPPED
              CANIVORE_NAME,
              16,
              27,
              false,
              new Debouncer(0),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      // FROM COMP BOT
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(90.0)
                          .withSupplyCurrentLimit(65.0))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          // TODO: BRINGUP: MAY NEED TO INVERT
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(70.0)
                          .withPeakReverseTorqueCurrent(70.0))),
          new DeployConfig(
              CANIVORE_NAME,
              20,

              // TODO: BRINGUP: VERIFY VALUES
              99,
              148,
              1.75,
              20.0,
              1,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  // UPDATED FOR OFFSEASON BOT
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              (72.0 / 12.0) * (72.0 / 18.0) * (24.0 / 12.0)))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))

                  // TODO: BRINGUP: SET TO ZERO AND TUNE
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKV(0.0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Arm_Cosine))),
          new ClimberConfig(
              CANIVORE_NAME,
              21,
              22,
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
                  // TODO: WAITING FOR CORRECT RATIO
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(100)
                          .withSupplyCurrentLimit(100)),
              // Cancoder
              new CANcoderConfiguration()
                  .withMagnetSensor(
                      new MagnetSensorConfigs()
                          // TODO: BRINGUP: RETUNE
                          .withMagnetOffset(-0.44921875)
                          .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                          .withAbsoluteSensorDiscontinuityPoint(0.5))),
          new SingulatorConfig(
              CANIVORE_NAME,
              28,
              29,
              9,
              8,
              new Debouncer(0.0),
              new Debouncer(0.0),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          // TODO: BRINGUP: MAY NEED TO INVERT
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast)),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          // TODO: BRINGUP: MAY NEED TO INVERT
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new ElevatorConfig(
              CANIVORE_NAME,
              15,
              0.0,
              0.0,
              0.0,
              49.5,
              0.0,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(60)
                          .withStatorCurrentLimit(60))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          // TODO: BRINGUP: MAY NEED TO INVERT
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(
                      new Slot0Configs()
                          // TODO: BRINGUP: REMOVE VALUES AND TUNE
                          .withKP(0.0)
                          .withKD(0.0)
                          .withKV(0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Elevator_Static))
                  // TODO: BRINGUP: REMOVE VALUES AND TUNE
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(9999999.0)
                          .withMotionMagicCruiseVelocity(9999999.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              // UPDATED FOR OFFSEASON BOT
                              (12.0 / 40.0) * (20.0 / 44.0) * (Math.PI * 1.881)))),
          new ArmConfig(
              RIO_CAN_NAME,
              18,
              -90,
              75,
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(NeutralModeValue.Brake)
                          .withInverted(InvertedValue.CounterClockwise_Positive))
                  // TODO: BRINGUP: REMOVE VALUES AND TUNE
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKV(0.0)
                          .withKD(0.0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Arm_Cosine))
                  // TODO: BRINGUP: REMOVE VALUES
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(999999.0)
                          .withMotionMagicCruiseVelocity(999999.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              // UPDATED FOR OFFSEASON BOT
                              (36.0 / 10.0) * (60.0 / 18.0) * (48.0 / 10.0)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(60.0)
                          .withStatorCurrentLimit(60.0)),
              // TODO: BRINGUP: RETUNE
              -90.0),
          new VisionConfig(
              0.005,
              0.8,
              // Translation: Positive X = Forward, Positive Y = Left, Positive Z = Up
              // Rotation: Positive X = Roll Right, Positive Y = Pitch Down, Positive Z = Yaw Left
              // Left Camera Config
              new CameraConfig(
                  LimelightModel.THREEG,
                  true,
                  0.1045038296,
                  -0.2494524094,
                  0.5819300782,
                  -20.0,
                  -60.0,
                  90.0),
              // Right Camera Config
              new CameraConfig(
                  LimelightModel.THREEG,
                  true,
                  0.0956578478,
                  0.2345115452,
                  0.5819625648,
                  -20.0,
                  60.0,
                  -90.0)),
          new LightsConfig(CANIVORE_NAME, 17),
          new SwerveConfig(new PhoenixPIDController(5.75, 0, 0), true, true, true));

  private CompConfig() {}
}
