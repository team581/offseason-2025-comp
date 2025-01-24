package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** A interpolated vision data set for each field, for all cameras. */
public enum InterpolatedVisionDataset {
  HOME(
      new CameraDataset(
          List.of(),
          List.of()),
          new CameraDataset(
          List.of(),
          List.of()),
          new CameraDataset(
          List.of(),
          List.of())
  );

  public final CameraDataset elevatorPurpleSet;
  public final CameraDataset frontCoralSet;
  public final CameraDataset backTagSet;

  InterpolatedVisionDataset(
      CameraDataset elevatorPurpleSet, CameraDataset frontCoralSet, CameraDataset backTagSet) {
    this.elevatorPurpleSet = elevatorPurpleSet;
    this.frontCoralSet = frontCoralSet;
    this.backTagSet = backTagSet;
  }
}
