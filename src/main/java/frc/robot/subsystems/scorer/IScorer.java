package frc.robot.subsystems.scorer;

import edu.wpi.first.math.geometry.Pose3d;

public interface IScorer {

  void periodic();

  boolean hasCoral();

  void intakeFromHP();

  void prepareToPlaceCoralOnBranch();

  void removeAlgaeFromBranch(Pose3d reefFaceToRemove);

  void moveScorerToDefaultPosition();

  void placeCoral();

  void homeElevator();

  void setElevatorTestPosition(double testPosition);

  void setPivotTestPosition(double testPosition);

  void setElevatorDutyCycle(double dutyCycle);

  void setPivotDutyCycle(double dutyCycle);

  void setEndEffectorDutyCycle(double dutyCycle);

  boolean isAtCollectPosition();

  boolean isAtDefaultPosition();

  boolean isSecuredToPlaceCoral();

  boolean hasPlaced();

  void setCoastScorer();

  void setBrakeScorer();
}
