package frc.robot.vision.interpolation;

import java.util.List;

/** A interpolated vision data set for each field, for all cameras. */
public enum InterpolatedVisionDataset {
  HOME(
      new CameraDataset(List.of(), List.of()),
      new CameraDataset(List.of(), List.of()),
      new CameraDataset(List.of(), List.of()),
      new CameraDataset(List.of(), List.of()));

  public final CameraDataset elevatorPurpleSet;
  public final CameraDataset frontCoralSet;
  public final CameraDataset backTagSet;
  public final CameraDataset baseTagSet;

  InterpolatedVisionDataset(
      CameraDataset elevatorPurpleSet,
      CameraDataset frontCoralSet,
      CameraDataset backTagSet,
      CameraDataset baseTagSet) {
    this.elevatorPurpleSet = elevatorPurpleSet;
    this.frontCoralSet = frontCoralSet;
    this.backTagSet = backTagSet;
    this.baseTagSet = baseTagSet;
  }
}
