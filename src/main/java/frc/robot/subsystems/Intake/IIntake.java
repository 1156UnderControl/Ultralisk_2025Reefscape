package frc.robot.subsystems.intake;

public interface IIntake {

  void intake();

  void expell();

  void goToIntakePosition();

  void goToSecuredPosition();

  boolean isAtSetPoint();

}
