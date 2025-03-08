package frc.robot.subsystems.climber;

public interface IClimber {

  void climb();

  boolean isAtIntakeCagePosition();

  boolean isAtClimbPosition();

  void lockClimber();

  void unlockClimber();

  void goToIntakeCagePosition();

  void goToStowPosition();

  void intakeCage();

  void stopClimberArm();

  void stopIntakingCage();

  boolean isCageCollected();

  void periodic();

  void setCoastClimber();

  void setBrakeClimber();

}
