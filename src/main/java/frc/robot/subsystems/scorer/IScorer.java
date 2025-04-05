package frc.robot.subsystems.scorer;

import java.util.function.Supplier;

import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightScore;
import frc.robot.constants.FieldConstants.ReefLevel;

public interface IScorer {

  void periodic();

  boolean hasCoral();

  boolean hasAlgae();

  void intakeFromHP();

  void stopIntakeFromHP();

  void placeCoral();

  void placeAlgae();

  void prepareToScoreAlgae();

  void prepareToPlaceCoralOnBranch();

  void prepareToPlaceCoralOnBranch(Supplier<Double> distanceToTargetPoseProvider);

  void collectAlgae();

  void holdAlgae();

  void stopEndEffector();

  void moveScorerToDefaultPosition();

  void setElevatorTestPosition(double testPosition);

  void setPivotTestPosition(double testPosition);

  void setElevatorDutyCycle(double dutyCycle);

  void setElevatorVoltage(double voltage);

  void setPivotDutyCycle(double dutyCycle);

  void setEndEffectorDutyCycle(double dutyCycle);

  boolean isAtCollectAlgaePosition();

  boolean isAtCollectCoralPosition();

  boolean isAtDefaultPosition();

  boolean isAtRemovePosition();

  boolean isSecuredToPlaceCoral();

  boolean isSecuredToScoreOnNet();

  boolean isElevatorInHighPosition();

  ReefLevel getTargetReefLevel();

  void setTargetBranchLevel(ReefLevel reefHeight);

  void setTargetAlgaeHeight(AlgaeHeightReef algaeHeight);

  void setTargetAlgaeScoreHeight(AlgaeHeightScore algaeHeightScore);

  void setCoastScorer();

  void setBrakeScorer();
}
