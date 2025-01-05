package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.vision.interpolation.CameraDataset;
import frc.robot.vision.interpolation.InterpolatedVision;
import java.util.Optional;

public class Limelight {
  private final String limelightTableName;
  private final String name;
  private CameraDataset interpolationData;
  private CameraStatus state = CameraStatus.ONLINE_NO_TAGS;
  private double limelightHeartbeat = -1;

  private final Timer limelightTimer = new Timer();

  public Limelight(String name, CameraDataset interpolationData) {
    limelightTableName = "limelight-" + name;
    this.name = name;
    this.interpolationData = interpolationData;
    limelightTimer.start();
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public Optional<VisionResult> getInterpolatedVisionResult() {
    var rawResult = getRawVisionResult();

    updateState(rawResult);

    if (rawResult.isEmpty()) {
      return Optional.empty();
    }

    Pose2d interpolatedPose =
        InterpolatedVision.interpolatePose(rawResult.get().pose(), interpolationData);
    return Optional.of(new VisionResult(interpolatedPose, rawResult.get().timestamp()));
  }

  private Optional<VisionResult> getRawVisionResult() {
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);

    if (estimatePose == null) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/RawLimelightPose", estimatePose.pose);

    if (estimatePose.tagCount == 0) {
      return Optional.empty();
    }

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (estimatePose.pose.getX() == 0.0 && estimatePose.pose.getY() == 0.0) {
      return Optional.empty();
    }

    return Optional.of(new VisionResult(estimatePose.pose, estimatePose.timestampSeconds));
  }

  private void updateState(Optional<VisionResult> rawResult) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");

    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(5)) {
      state = CameraStatus.OFFLINE;
      return;
    }

    if (!rawResult.isEmpty()) {
      state = CameraStatus.SEES_TAGS;
      return;
    }
    state = CameraStatus.ONLINE_NO_TAGS;
  }

  public CameraStatus getState() {
    return state;
  }
}
