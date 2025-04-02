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

  void preparetoScoreAlgae();

  void prepareToPlaceCoralOnBranch();

  void prepareToPlaceCoralOnBranch(Supplier<Double> distanceToTargetPoseProvider);

  void removeAlgaeFromBranch();

  void removeAlgaeEndEffector();

  void stopEndEffector();

  void moveScorerToDefaultPosition();

  void moveScorerToAlgaeDefaultPosition();

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

  boolean isElevatorInHighPosition();

  ReefLevel getTargetReefLevel();

  void setTargetBranchLevel(ReefLevel reefHeight);

  void setTargetAlgaeHeight(AlgaeHeightReef algaeHeight);

  void setTargetAlgaeScoreHeight(AlgaeHeightScore algaeHeightScore);

  void collectAlgaeFromReef();

  void collectAlgaeFromGround();

  void setCoastScorer();

  void setBrakeScorer();

  void setAngle180();

  void setAngle10();
}
