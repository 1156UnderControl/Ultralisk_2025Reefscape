package frc.robot.subsystems.intake;

public interface IIntake {

  void intake();

  void expell();

  void stopIntake();

  boolean isCoralDetected();

  void periodic();
}
