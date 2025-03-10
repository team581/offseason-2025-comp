package frc.robot.elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.Points;

public enum CoralStation {
  PROCESSOR_SIDE_RED(37.25, Points.RIGHT_CORAL_STATION.redPose),
  PROCESSOR_SIDE_BLUE(37.25, Points.RIGHT_CORAL_STATION.bluePose),
  NON_PROCESSOR_SIDE_RED(37.25, Points.LEFT_CORAL_STATION.redPose),
  NON_PROCESSOR_SIDE_BLUE(37.25, Points.LEFT_CORAL_STATION.bluePose);

  /** The height in inches from the carpet to the bottom of the coral station plastic. */
  private static final double IDEAL_HEIGHT = 37.25;

  /** The number of inches to add to the elevator height to intake from this station. */
  public final double offset;

  public final Pose2d backLoadPose;
  public final Pose2d frontLoadPose;

  CoralStation(double measuredHeight, Pose2d pose) {
    offset = IDEAL_HEIGHT - measuredHeight;
    this.backLoadPose = pose;
    this.frontLoadPose = new Pose2d(pose.getTranslation(), Rotation2d.k180deg);
  }
}
