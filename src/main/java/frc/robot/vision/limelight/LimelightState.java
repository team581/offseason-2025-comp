package frc.robot.vision.limelight;

public enum LimelightState {
  CORAL(2),
  TAGS(1),
  PURPLE(3),
  REEF_TAGS(4);

  final int pipelineIndex;

  LimelightState(int index) {
    this.pipelineIndex = index;
  }
}
