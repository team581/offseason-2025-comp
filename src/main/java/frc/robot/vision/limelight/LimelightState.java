package frc.robot.vision.limelight;

public enum LimelightState {
  TAGS(1),
  CORAL(2),
  PURPLE(3),
  REEF_TAGS(1);

  final int pipelineIndex;

  LimelightState(int index) {
    this.pipelineIndex = index;
  }
}
