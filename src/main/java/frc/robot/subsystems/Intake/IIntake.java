package frc.robot.subsystems.Intake;

public interface IIntake {

  void intake();

  void expell();

  void goToIntakePosition();

  void goToSecuredPosition();

  boolean isAtSetPoint();

}
