package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.math.MathHelpers;
import com.team581.trailblazer.LocalizationBase;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.ImuSubsystem;
import frc.robot.odometry.CustomOdometry;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.results.TagResult;

public class LocalizationSubsystem extends StateMachineSubsystem<LocalizationState>
    implements LocalizationBase {
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;

  private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
  private Pose2d robotPose = Pose2d.kZero;
  private static final DoubleSubscriber LATENCY_CONSTANT =
      DogLog.tunable("Localization/VisionLatencyConstantMS", 20.0);
  // Currently using default std devs for odometry
  private static final Vector<N3> ODOMETRY_STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> ODOMETRY_VISION_MEASURMENT_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);

  public LocalizationSubsystem(
      ImuSubsystem imu,
      VisionSubsystem vision,
      SwerveSubsystem swerve,
      SwerveDriveKinematics kinematics,
      CustomOdometry customOdometry) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;

    this.poseEstimator =
        new PoseEstimator<>(
            kinematics, customOdometry, ODOMETRY_STATE_STD_DEVS, ODOMETRY_VISION_MEASURMENT_STD_DEVS);

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo180",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(180))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo0",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(0))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo90",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(90))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo270",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(270))).ignoringDisable(true));
    }
  }

  @Override
  protected void collectInputs() {
    vision
        .getLeftFrontTagResult()
        .or(vision::getLeftBackTagResult)
        .ifPresent(this::ingestTagResult);
    vision.getRightTagResult().ifPresent(this::ingestTagResult);
    vision.getGamePieceTagResult().ifPresent(this::ingestTagResult);

    if (FeatureFlags.CUSTOM_ODOMETRY.getAsBoolean()) {
      robotPose = poseEstimator.getEstimatedPosition();
    } else {
      robotPose = swerve.drivetrain.getState().Pose;
    }
  }

  @Override
  public Pose2d getPose() {
    return robotPose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);

    if (FeatureFlags.CUSTOM_ODOMETRY.getAsBoolean()) {
      return poseEstimator.sampleAt(newTimestamp).orElseGet(this::getPose);
    } else {
      return swerve.drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
    }
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  @Override
  public void whileInState(LocalizationState currentState) {
    DogLog.log("Localization/EstimatedPose", getPose());
    var swerveState = swerve.drivetrain.getState();

    if (FeatureFlags.CUSTOM_ODOMETRY.getAsBoolean()) {
      poseEstimator.update(swerveState.RawHeading, swerveState.ModulePositions);
    }
  }

  private void ingestTagResult(TagResult result) {
    var visionPose = result.pose();

    if (!vision.seenTagRecentlyForReset() && FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
      resetPose(visionPose);
    }

    if (FeatureFlags.CUSTOM_ODOMETRY.getAsBoolean()) {
      poseEstimator.addVisionMeasurement(visionPose, result.timestamp(), result.standardDevs());
    } else {
      swerve.drivetrain.addVisionMeasurement(
        visionPose,
        Utils.fpgaToCurrentTime(result.timestamp() - (LATENCY_CONSTANT.getAsDouble() / 1000)),
        result.standardDevs());
    }
    DogLog.log("Localization/VisionPose", visionPose);
  }

  private void resetGyro(Rotation2d gyroAngle) {
    imu.setAngle(gyroAngle.getDegrees());

    if (FeatureFlags.CUSTOM_ODOMETRY.getAsBoolean()) {
      poseEstimator.resetRotation(gyroAngle);
    } else {
      swerve.drivetrain.resetRotation(gyroAngle);
    }
  }

  public void resetPose(Pose2d estimatedPose) {
    // Reset the gyro when requested in teleop
    // Otherwise, if we are in auto, only reset it if we aren't already at the correct heading
    if (DriverStation.isTeleop()
        || !MathUtil.isNear(
            estimatedPose.getRotation().getDegrees(), imu.getRobotHeading(), 1.5, -180, 180)) {
      imu.setAngle(estimatedPose.getRotation().getDegrees());
    }

    if (FeatureFlags.CUSTOM_ODOMETRY.getAsBoolean()) {
      poseEstimator.resetPose(estimatedPose);
    } else {
      swerve.drivetrain.resetPose(estimatedPose);
    }
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees((FmsUtil.isRedAlliance() ? 180 : 0))));
  }
}
