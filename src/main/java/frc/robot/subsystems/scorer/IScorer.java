package frc.robot.subsystems.scorer;

public interface IScorer {

  void periodic();

  void detectthecollect();

  boolean hasCoral();

  void intake();

  void intakeFromHP();

  void prepareToPlaceL1();

  void prepareToPlaceL2();

  void prepareToPlaceL3();

  void prepareToPlaceL4();

  void place();

  void expell();

  boolean isSecuredToPlace();

  boolean hasPlaced();

  void stopIntaking();

  boolean isAtSetPoint();

}
