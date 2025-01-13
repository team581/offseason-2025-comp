package frc.robot.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.results.TagResult;

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  private static final Vector<N3> VISION_STD_DEVS =
      VecBuilder.fill(
          RobotConfig.get().vision().xyStdDev(),
          RobotConfig.get().vision().xyStdDev(),
          RobotConfig.get().vision().thetaStdDev());
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private double lastAddedVisionTimestamp = 0;
  private List<TagResult> latestResult = new ArrayList<>();

  public LocalizationSubsystem(ImuSubsystem imu, VisionSubsystem vision, SwerveSubsystem swerve) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;
  }

  @Override
  protected void collectInputs() {
    latestResult = vision.getInterpolatedVisionResult();
  }

  public Pose2d getPose() {
    return swerve.getDrivetrainState().Pose;
  }

  public Optional<Pose2d> getPose(double timestamp) {
    return swerve.drivetrain.samplePoseAt(timestamp);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    for (var results : latestResult) {
      Pose2d visionPose = results.pose();

      double visionTimestamp = results.timestamp();

      if (visionTimestamp == lastAddedVisionTimestamp) {
        // Don't add the same vision pose over and over
      } else {
        swerve.drivetrain.addVisionMeasurement(visionPose, visionTimestamp, VISION_STD_DEVS);
        lastAddedVisionTimestamp = visionTimestamp;
      }

    }

    DogLog.log("Localization/EstimatedPose", getPose());
  }

  private void resetGyro(double gyroAngle) {
    Pose2d estimatedPose =
        new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(gyroAngle));
    resetPose(estimatedPose);
  }

  public void resetPose(Pose2d estimatedPose) {
    imu.setAngle(estimatedPose.getRotation().getDegrees());
    swerve.drivetrain.resetPose(estimatedPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(() -> resetGyro(FmsSubsystem.isRedAlliance() ? 0 : 180));
  }

  /**
   * Get the field relative angle the robot should face in order to be looking directly at the
   * target pose.
   */
  public DistanceAngle getFieldRelativeDistanceAngleToPose(Pose2d target) {
    DistanceAngle distanceAngleToSpeaker = distanceAngleToTarget(target, getPose());

    return distanceAngleToSpeaker;
  }

  public static DistanceAngle distanceAngleToTarget(Pose2d target, Pose2d current) {
    double distance = target.getTranslation().getDistance(current.getTranslation());
    double angle =
        Units.radiansToDegrees(
            Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));

    return new DistanceAngle(distance, angle, false);
  }
}
