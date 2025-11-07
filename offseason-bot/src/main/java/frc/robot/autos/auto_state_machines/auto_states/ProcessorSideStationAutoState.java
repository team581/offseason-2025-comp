package frc.robot.autos.auto_state_machines.auto_states;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.Points;

public enum ProcessorSideStationAutoState {
  INTAKING(Points.GROUND_INTAKE_PROCESSOR_SIDE_STATION),
  SCORE(Points.GROUND_INTAKE_PROCESSOR_SIDE_STATION);

  public final Points point;

  public Pose2d getPose() {
    return this.point.getPose();
  }

  private ProcessorSideStationAutoState(Points point) {
    this.point = point;
  }
}
