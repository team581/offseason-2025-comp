package frc.robot.auto_align;

public enum AutoAlignState {
  // Safe point, not ready to score
  EXPLICIT_SAFE_WAITING,
  // Safe point, ready to score
  EXPLICIT_SAFE,
  // Left chosen, centering with pipe
  EXPLICIT_LEFT_CENTER,
  // Right chosen, centering with pipe
  EXPLICIT_RIGHT_CENTER,
  // Left chosen, waiting to score
  EXPLICIT_LEFT_WAITING,
  // Right chosen, waiting to score
  EXPLICIT_RIGHT_WAITING,
  // Scoring on left pipe
  LEFT_PIPE,
  // Scoring on right pipe
  RIGHT_PIPE,
  // Centering with best pipe
  BEST_PIPE_CENTER,
  // Waiting to score on best pipe
  BEST_PIPE_WAITING,
  // Scoring on best pipe
  BEST_PIPE,
  // Backing away from closest pipe after scoring
  PIPE_BACKUP,
  // Centering with algae
  ALGAE_CENTER,
  // Waiting to intake algae
  ALGAE_WAITING,
  // Intaking algae
  ALGAE_INTAKE,
  // Backing away from reef after intake
  ALGAE_BACKUP;
}
