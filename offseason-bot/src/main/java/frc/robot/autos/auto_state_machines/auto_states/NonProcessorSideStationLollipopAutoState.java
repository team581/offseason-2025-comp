package frc.robot.autos.auto_state_machines.auto_states;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.Points;

public enum NonProcessorSideStationLollipopAutoState {
  INTAKING(Points.GROUND_INTAKE_NON_PROCESSOR_SIDE_STATION),
  SCORE(Points.GROUND_INTAKE_NON_PROCESSOR_SIDE_STATION),
  LOLLIPOP_2(Points.LOLLIPOP_2);

  public final Points point;

  public Pose2d getPose() {
    return this.point.getPose();
  }

  private NonProcessorSideStationLollipopAutoState(Points point) {
    this.point = point;
  }
}
