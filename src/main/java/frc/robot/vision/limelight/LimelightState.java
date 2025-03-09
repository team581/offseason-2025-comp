package frc.robot.vision.limelight;

public enum LimelightState {
  TAGS(1),
  CLOSEST_REEF_TAG(1),
  STATION_TAGS(1),
  CORAL(2),
  ALGAE(4);

  final int pipelineIndex;

  LimelightState(int index) {
    this.pipelineIndex = index;
  }
}
