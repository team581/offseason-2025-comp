package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.PivotConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.config.RobotConfig.WristConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;

class PracticeConfig {
  private static final String CANIVORE_NAME = TunerConstants.kCANBus.getName();
  private static final String RIO_CAN_NAME = "rio";

  public static final RobotConfig practiceBot =
      new RobotConfig(
          "practice",
          new ElevatorConfig(
              CANIVORE_NAME,
              15,
              16,
              // Sensor to mechanism ratio is the gear ratio multiplied by the sprocket circumfrence
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(new Slot0Configs().withKP(0.0).withKV(0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              ((50.0 / 14.0) * (50.0 / 36.0)) * (2.0 / 1.0) * (Math.PI * 1.274))),
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(new Slot0Configs().withKP(0.0).withKV(0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              ((50.0 / 14.0) * (50.0 / 36.0)) * (2.0 / 1.0) * (Math.PI * 1.274))),
              0,
              0,
              68,
              0.25),
          new IntakeConfig(
              RIO_CAN_NAME,
              20,
              21,
              new Debouncer(0.0, DebounceType.kBoth),
              new Debouncer(0.0, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(15))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast)),
              new TalonFXConfiguration()
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(15))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new SwerveConfig(
              new PhoenixPIDController(10, 0, 1),
              true,
              true,
              true,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(80))
                  .withOpenLoopRamps(
                      new OpenLoopRampsConfigs()
                          .withDutyCycleOpenLoopRampPeriod(0.01)
                          .withVoltageOpenLoopRampPeriod(0.01)
                          .withTorqueOpenLoopRampPeriod(0.01))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
                  .withMotorOutput(
                      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          // Swerve azimuth does not require much torque output, so we can set a
                          // relatively low stator current limit to help avoid brownouts without
                          // impacting performance.
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(50)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(80))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
                  .withMotorOutput(
                      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))),
          new VisionConfig(4, 0.4, 0.4, InterpolatedVisionDataset.MADTOWN),
          new WristConfig(
              RIO_CAN_NAME,
              22,
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
                  .withSlot0(new Slot0Configs().withKP(0.0).withKV(0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((64 / 8) * (50 / 18) * (36 / 12)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimitEnable(true)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(25.0)
                          .withStatorCurrentLimit(25.0)),
              0,
              180),
          new PivotConfig(
              RIO_CAN_NAME,
              23,
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(new Slot0Configs().withKP(0.0).withKV(0))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(2).withPeakReverseVoltage(2))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(10)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(10)),
              // TODO: Slides recommend 10A threshold
              0,
              0),
          new ClimberConfig(
              CANIVORE_NAME,
              24,
              25,
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(10)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(10)),
              new CANcoderConfiguration()
                  .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0))),
          new LightsConfig(RIO_CAN_NAME, 18));

  private PracticeConfig() {}
}
