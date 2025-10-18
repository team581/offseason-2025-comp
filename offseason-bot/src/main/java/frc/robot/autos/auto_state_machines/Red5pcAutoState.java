package frc.robot.autos.auto_state_machines;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.Points;

public enum Red5pcAutoState {
  IDLE(Points.START_R2_AND_B2.getPose()), // (wait for auto to start)
  // -Pose R2

  INTAKING(Points.GROUND_INTAKE_LEFT_STATION.getPose()),
  // -Pose by station, robot state intaking
  PRE_INTAKING(Points.PRE_GROUND_INTAKE_LEFT_STATION.getPose()),
  // -Pose before getting to station, robot state idle
  POST_INTAKING(Points.PRE_GROUND_INTAKE_LEFT_STATION.getPose()),
  // -Pose before getting to station, robot state intaking
  LOLLIPOP_2(Points.LOLLIPOP_2.getPose()),
  // -LP 2 pose, intaking state
  PRE_LOLLIPOP_2(Points.PRE_LOLLIPOP_2.getPose()),
  // -away from LP 2 pose, idle state
  POST_LOLLIPOP_2(Points.PRE_LOLLIPOP_2.getPose()),
  // -away from LP 2 pose, intaking state

  I_L4_LINEUP(Points.PRE_I_L4.getPose()),
  J_L4_LINEUP(Points.PRE_J_L4.getPose()),
  // -robot idle state, a little before I/J pose
  K_L4_LINEUP(Points.PRE_K_L4.getPose()),
  L_L4_LINEUP(Points.PRE_L_L4.getPose()),
  // -robot intaking state (until singulate coral), a little before K/L pose
  A_L4_LINEUP(Points.PRE_A_L4.getPose()),
  B_L4_LINEUP(Points.PRE_B_L4.getPose()),
  // -robot intaking state (until singulate coral), a little before A/B pose

  I_L4_PREPARE(Points.I_L4_POSE.getPose()),
  J_L4_PREPARE(Points.J_L4_POSE.getPose()),
  // -move superstructure for scoring, I/J pose
  K_L4_PREPARE(Points.K_L_POSE.getPose()),
  L_L4_PREPARE(Points.LEFT_CORAL_STATION.getPose()),
  // -move superstructure for scoring, K/L pose
  A_L4_PREPARE(Points.A_L4_POSE.getPose()),
  B_L4_PREPARE(Points.B_L4_POSE.getPose()), // -move superstructure for scoring, A/B pose

  I_L4_SCORE(Points.I_L4_POSE.getPose()),
  J_L4_SCORE(Points.J_L4_POSE.getPose()),
  K_L4_SCORE(Points.K_L_POSE.getPose()),
  L_L4_SCORE(Points.L_L4_POSE.getPose()),
  A_L4_SCORE(Points.A_L4_POSE.getPose()),
  B_L4_SCORE(Points.B_L4_POSE.getPose()),

  // ll-robot score state, reef pipe score pose

  I_L4_POST_SCORING(Points.PRE_I_L4.getPose()),
  J_L4_POST_SCORING(Points.PRE_J_L4.getPose()),
  // -robot stow state (i think stowed is in handoff position), a little before I/J pose
  K_L4_POST_SCORING(Points.PRE_K_L4.getPose()),
  L_L4_POST_SCORING(Points.PRE_L_L4.getPose()),
  // -robot stow state, a little before K/L pose
  A_L4_POST_SCORING(Points.PRE_A_L4.getPose()),
  B_L4_POST_SCORING(Points.PRE_B_L4.getPose());
  // -robot stow state, a little before A/B pose
  public final Pose2d pose;

  public Pose2d getPose() {
    return this.pose;
  }

  private Red5pcAutoState(Pose2d pose) {
    this.pose = pose;
  }
}
