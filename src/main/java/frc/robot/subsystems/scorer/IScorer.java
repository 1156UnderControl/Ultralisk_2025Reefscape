package frc.robot.subsystems.scorer;

import frc.robot.constants.FieldConstants.AlgaeHeight;
import frc.robot.constants.FieldConstants.ReefLevel;

public interface IScorer {

  void periodic();

  boolean hasCoral();

  void intakeFromHP();

  void stopIntakeFromHP();

  void prepareToPlaceCoralOnBranch();

  void prepareToPlaceCoralOnBranch(ReefLevel reefLevel);

  void removeAlgaeFromBranch();

  void moveScorerToDefaultPosition();

  void placeCoral();

  void homeElevator();

  void setElevatorTestPosition(double testPosition);

  void setPivotTestPosition(double testPosition);

  void setElevatorDutyCycle(double dutyCycle);

  void setElevatorVoltage(double voltage);

  void setPivotDutyCycle(double dutyCycle);

  void setEndEffectorDutyCycle(double dutyCycle);

  boolean isAtCollectPosition();

  boolean isAtDefaultPosition();

  boolean isAtRemovePosition();

  boolean isSecuredToPlaceCoral();

  void setTargetBranchLevel(ReefLevel reefHeight);

  void setTargetAlgaeHeight(AlgaeHeight algaeHeight);

  boolean hasPlaced();

  void setCoastScorer();

  void setBrakeScorer();
}
