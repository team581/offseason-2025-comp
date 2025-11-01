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
import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.ClawConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.config.RobotConfig.WristConfig;

class CompConfig {

  private static final String RIO_CAN_NAME = "rio";

  public static final RobotConfig COMPETITION_BOT =
      new RobotConfig(
          new ClawConfig(
              RIO_CAN_NAME,
              16,
              30.0,
              5.0,
              3.0,
              new Debouncer(0),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(90.0)
                          .withSupplyCurrentLimit(65.0))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new ClimberConfig(
              RIO_CAN_NAME,
              18,
              19,
              20,
              21,
              -55.0,
              100.0,
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
                          .withMagnetOffset(0.42529296875)
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
          // TODO: add radius
          new ElevatorConfig(
              RIO_CAN_NAME,
              15,
              0.0,
              0.0,
              0.0,
              99.75,
              0.0,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(60)
                          .withStatorCurrentLimit(60))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKD(0.0)
                          .withKV(0.0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Elevator_Static))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(0.0)
                          .withMotionMagicCruiseVelocity(0.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((7.886/10.0)*(16.0 / 36.0) * (20.0 / 44.0) * (Math.PI * 1.881)))),
          new WristConfig(
              RIO_CAN_NAME,
              17,
              151.0, // TODO: 151 actually, 146 to be safe. Change after bringup
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
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((92.0 / 1.0)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(60.0)
                          .withStatorCurrentLimit(60.0))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)),
              150.295),
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

              // Limelight position relative to robot bellypan center (meters)
              // Limelight class takes this in to set position from code
              new CameraConfig(LimelightModel.THREE, true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
          new SwerveConfig(new PhoenixPIDController(5.75, 0, 0), true, false, false));

  private CompConfig() {}
}
