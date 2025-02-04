package frc.robot.subsystems.climber;

public interface IClimber {

  void climb();

  void isAtSetPoint();

  void climbDeep();

  void release();

  void stop();

  void periodic();
}
