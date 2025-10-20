package frc.robot.vision.limelight;

public enum LimelightState {
  OFF(1),
  TAGS(1),
  CORAL(2),
  ALGAE(4),
  HANDOFF(5);

  final int pipelineIndex;

  LimelightState(int index) {
    this.pipelineIndex = index;
  }
}
