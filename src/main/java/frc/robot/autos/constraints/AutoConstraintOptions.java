package frc.robot.autos.constraints;

public record AutoConstraintOptions(
    /** Whether collision avoidance should be enabled. */
    boolean collisionAvoidance,
    /** Max linear velocity allowed in meters per second. Set to 0 to disable. */
    double maxLinearVelocity,
    /** Max angular velocity allowed in radians per second. Set to 0 to disable. */
    double maxAngularVelocity,
    /** Max linear acceleration allowed in meters per second squared. Set to 0 to disable. */
    double maxLinearAcceleration,
    /** Max angular acceleration allowed in radians per second squared. Set to 0 to disable. */
    double maxAngularAcceleration) {
  /** Default constraint options to use if no point or segment specific options are set. */
  public AutoConstraintOptions() {
    this(false, 0, 0, 0, 0);
  }

  public AutoConstraintOptions withCollisionAvoidance(boolean collisionAvoidance) {
    return new AutoConstraintOptions(
        collisionAvoidance,
        maxLinearVelocity(),
        maxAngularVelocity(),
        maxLinearAcceleration(),
        maxAngularAcceleration());
  }

  public AutoConstraintOptions withMaxLinearVelocity(double maxLinearVelocity) {
    return new AutoConstraintOptions(
        collisionAvoidance(),
        maxLinearVelocity,
        maxAngularVelocity(),
        maxLinearAcceleration(),
        maxAngularAcceleration());
  }

  public AutoConstraintOptions withMaxAngularVelocity(double maxAngularVelocity) {
    return new AutoConstraintOptions(
        collisionAvoidance(),
        maxLinearVelocity(),
        maxAngularVelocity,
        maxLinearAcceleration(),
        maxAngularAcceleration());
  }

  public AutoConstraintOptions withMaxLinearAcceleration(double maxLinearAcceleration) {
    return new AutoConstraintOptions(
        collisionAvoidance(),
        maxLinearVelocity(),
        maxAngularVelocity(),
        maxLinearAcceleration,
        maxAngularAcceleration());
  }

  public AutoConstraintOptions withMaxAngularAcceleration(double maxAngularAcceleration) {
    return new AutoConstraintOptions(
        collisionAvoidance(),
        maxLinearVelocity(),
        maxAngularVelocity(),
        maxLinearAcceleration(),
        maxAngularAcceleration);
  }
}
