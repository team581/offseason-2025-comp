package frc.robot.vision.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class LimelightPositions {
  // Translation: Positive X = Forward, Positive Y = Left, Positive Z = Up
  // Rotation: Positive X = Roll Right, Positive Y = Pitch Down, Positive Z = Yaw Left
  public static final Pose3d LEFT_FRONT_LIMELIGHT = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  public static final Pose3d LEFT_BACK_LIMELIGHT = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  public static final Pose3d RIGHT_LIMELIGHT = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
}
