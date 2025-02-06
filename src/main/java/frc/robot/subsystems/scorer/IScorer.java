package frc.robot.subsystems.scorer;

public interface IScorer {

  void periodic();

  boolean hasCoral();

  void intake();

  void intakeFromHP();

  void prepareToPlaceCoral();

  void place();

  void expell();

  boolean isSecuredToPlace();

  boolean hasPlaced();

  void stopIntaking();

  boolean isAtSetPoint();

}
