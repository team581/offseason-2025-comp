package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** A interpolated vision data set for each field, for all cameras. */
public enum InterpolatedVisionDataset {
  HOME(
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.201, 5.702), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(12.9895, 5.522),
                  new Translation2d(12.965, 5.655),
                  "PODIUM_SPEAKER_INTERSECTION")),
          List.of()),
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.202, 5.713), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(12.9895, 5.522),
                  new Translation2d(12.87, 5.581),
                  "PODIUM_SPEAKER_INTERSECTION")),
          List.of()),
      new CameraDataset(List.of(), List.of())),
  // bellarmine is not tested
  BELLARMINE(
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.125, 5.581), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(13.103, 5.560),
                  "PODIUM_SPEAKER_INTERSECTION"),
              new VisionInterpolationData(
                  new Translation2d(11.059, 6.842),
                  new Translation2d(11.18, 6.932),
                  "WING_LINE_MIDDLE"),
              new VisionInterpolationData(
                  new Translation2d(13.799, 4.202),
                  new Translation2d(13.67, 4.106),
                  "FRONT_PODIUM_MIDDLE")),
          List.of()),
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.125, 5.581), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(13.103, 5.560),
                  "PODIUM_SPEAKER_INTERSECTION"),
              new VisionInterpolationData(
                  new Translation2d(11.059, 6.842),
                  new Translation2d(11.18, 6.932),
                  "WING_LINE_MIDDLE"),
              new VisionInterpolationData(
                  new Translation2d(13.799, 4.202),
                  new Translation2d(13.67, 4.106),
                  "FRONT_PODIUM_MIDDLE")),
          List.of()),
      new CameraDataset(List.of(), List.of())),
  MADTOWN(
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522),
                  new Translation2d(15.15, 5.57),
                  "RED_SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(13.33, 5.56),
                  "RED_PODIUM_SPEAKER_INTERSECTION")),
          List.of(
              new VisionInterpolationData(
                  new Translation2d(1.315, 5.522), new Translation2d(1.38, 5.54), "BLUE_SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(3.144, 5.522),
                  new Translation2d(3.2, 5.42),
                  "BLUE_PODIUM_SPEAKER_INTERSECTION"))),
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522),
                  new Translation2d(15.15, 5.58),
                  "RED_SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(13.30, 5.54),
                  "RED_PODIUM_SPEAKER_INTERSECTION")),
          List.of(
              new VisionInterpolationData(
                  new Translation2d(1.315, 5.522), new Translation2d(1.41, 5.53), "BLUE_SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(3.144, 5.522),
                  new Translation2d(3.24, 5.47),
                  "BLUE_PODIUM_SPEAKER_INTERSECTION"))),
      new CameraDataset(List.of(), List.of()));

  public final CameraDataset topSet;
  public final CameraDataset bottomSet;
  public final CameraDataset backSet;

  InterpolatedVisionDataset(CameraDataset top, CameraDataset bottom, CameraDataset back) {
    this.topSet = top;
    this.bottomSet = bottom;
    this.backSet = back;
  }
}
