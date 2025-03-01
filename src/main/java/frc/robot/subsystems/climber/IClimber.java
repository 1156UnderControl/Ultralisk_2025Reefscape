package frc.robot.subsystems.climber;

public interface IClimber {

  void climb();

  boolean isAtRaisedPosition();

  boolean isAtClimbPosition();

  void raiseClimber();

  void intakeCage();

  void stopIntakingCage();

  boolean isCageCollected();

  void periodic();

  void setCoastClimber();

  void setBrakeClimber();

  void setArmDutyCycle(double dutyCicle);
}
