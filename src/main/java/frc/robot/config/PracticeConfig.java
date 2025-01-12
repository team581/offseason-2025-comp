package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.PivotConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.config.RobotConfig.WristConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;

class PracticeConfig {
  private static final String CANIVORE_NAME = TunerConstants.kCANBus.getName();
  private static final String RIO_CAN_NAME = "rio";

  private static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP =
      new ClosedLoopRampsConfigs()
          .withDutyCycleClosedLoopRampPeriod(0.04)
          .withTorqueClosedLoopRampPeriod(0.04)
          .withVoltageClosedLoopRampPeriod(0.04);
  private static final OpenLoopRampsConfigs OPEN_LOOP_RAMP =
      new OpenLoopRampsConfigs()
          .withDutyCycleOpenLoopRampPeriod(0.04)
          .withTorqueOpenLoopRampPeriod(0.04)
          .withVoltageOpenLoopRampPeriod(0.04);

  public static final RobotConfig competitionBot =
      new RobotConfig(
          "competition",
          new ElevatorConfig(
              // TODO: Get actual Values
              CANIVORE_NAME,
              14,
              15,
              new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(0.0).withKV(0)),
              new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(0.0).withKV(0)),
              0,
              0,
              68,
              0,
              0.25),
          new IntakeConfig(
              RIO_CAN_NAME,
              3,
              4,
              16,
              17,
              new Debouncer(0.0, DebounceType.kBoth),
              new Debouncer(0.0, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(0))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)),
              new TalonFXConfiguration()
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(0))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive))),
          new SwerveConfig(
              new PhoenixPIDController(10, 0, 1),
              true,
              true,
              true,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(75)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(55)
                          .withSupplyCurrentLowerTime(0.25))
                  .withOpenLoopRamps(
                      new OpenLoopRampsConfigs()
                          .withDutyCycleOpenLoopRampPeriod(0.25)
                          .withVoltageOpenLoopRampPeriod(0.25))
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
                          .withStatorCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(60)
                          .withSupplyCurrentLowerTime(0.2))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
                  .withMotorOutput(
                      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))),
          new VisionConfig(4, 0.4, 0.4, InterpolatedVisionDataset.MADTOWN),
          new WristConfig(
              RIO_CAN_NAME,
              5,
              new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(0.0).withKV(0)),
              0,
              180),
          new PivotConfig(
              RIO_CAN_NAME,
              6,
              new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(0.0).withKV(0)),
              0,
              0));

  private PracticeConfig() {}
}
