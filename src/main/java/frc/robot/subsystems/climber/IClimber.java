package frc.robot.subsystems.climber;

public interface IClimber {

  void climb();

  void isAtSetPoint();

  void raiseClimber();

  void intakeCage();

  void stopIntakingCage();

  boolean isCageCollected();

  void periodic();
}
