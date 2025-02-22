package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.AlgaeHeight;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.constants.PivotConstants;

public class ScorerSubsystem implements IScorer {

  private static ScorerSubsystem instance;
  private IMotor elevatorMotorLeader = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, "ELEVATOR_MASTER");
  private IMotor elevatorMotorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor,
      "ELEVATOR_FOLLOWER");

  private final IMotor pivotMotor = new SparkMAXMotor(PivotConstants.ID_pivotMotor, false, "PIVOT");
  private final IMotor endEffectorMotor = new SparkMAXMotor(EndEffectorConstants.ID_endEffectorMotor, "END_EFFECTOR");

  private boolean hasCoral = false;
  private boolean elevatorHasHomed = false;
  private double goalElevator = ElevatorConstants.ZERO_POSITION_IN_METERS_FROM_GROUND;
  private double goalPivot = 0;

  StabilizeChecker motorNotMoving = new StabilizeChecker(0.4);

  CustomBooleanLogger correctingPivot = new CustomBooleanLogger("/ScorerSubsystem/Correcting Pivot");

  @Logged(name = "State", importance = Importance.CRITICAL)
  private String state = "START";

  @Logged(name = "Target Branch Height", importance = Importance.INFO)
  private String branchHeightTarget = "NONE";

  @Logged(name = "Target Reef Face To Remove Algae", importance = Importance.INFO)
  private String reefFaceTarget = "NONE";

  CustomBooleanLogger hasCoralLog = new CustomBooleanLogger("/ScorerSubsystem/hasCoral");

  CustomBooleanLogger hasAcceleratedLog = new CustomBooleanLogger("/ScorerSubsystem/hasAccelerated");

  CustomBooleanLogger elevatorStoppedByPivotLimit = new CustomBooleanLogger(
      "/ScorerSubsystem/elevatorStoppedByPivotLimit");

  CustomBooleanLogger pivotStoppedByElevatorLimit = new CustomBooleanLogger(
      "/ScorerSubsystem/pivotStoppedByElevatorLimit");

  private ReefHeight targetReefHeight = ReefHeight.L1;

  private AlgaeHeight targetAlgaeHeight = AlgaeHeight.LOW;

  private boolean manualControl = false;

  private boolean endEffectorAccelerated = false;

  private double lastGoalPivot = goalPivot;

  public static ScorerSubsystem getInstance() {
    if (instance == null) {
      instance = new ScorerSubsystem();
    }
    return instance;
  }

  private ScorerSubsystem() {
    setConfigsElevator();
    setConfigsPivot();
    setConfigsEndEffector();
  }

  private void setConfigsElevator() {
    elevatorMotorLeader.setMotorBrake(true);
    elevatorMotorFollower.setMotorBrake(true);
    elevatorMotorFollower.setFollower(ElevatorConstants.ID_elevatorLeaderMotor, true);
    elevatorMotorLeader.setPositionFactor(ElevatorConstants.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS);
    elevatorMotorLeader.setVelocityFactor(ElevatorConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND);
    elevatorMotorLeader.configurePIDF(
        ElevatorConstants.tunning_values_elevator.PID.P,
        ElevatorConstants.tunning_values_elevator.PID.I,
        ElevatorConstants.tunning_values_elevator.PID.D,
        0,
        ElevatorConstants.tunning_values_elevator.PID.IZone);
    elevatorMotorFollower.burnFlash();
    elevatorMotorLeader.burnFlash();
    elevatorMotorLeader.setPosition(ElevatorConstants.ZERO_POSITION_IN_METERS_FROM_GROUND);
  }

  private void setConfigsPivot() {
    pivotMotor.setInverted(false);
    pivotMotor.configExternalEncoder();
    pivotMotor.setInvertedEncoder(true);
    pivotMotor.setMotorBrake(true);
    pivotMotor.setPositionFactor(PivotConstants.POSITION_FACTOR_ROTOR_ROTATION_TO_MECHANISM_DEGREES);
    pivotMotor.setPositionFactorExternalEncoder(PivotConstants.POSITION_FACTOR_MECHANISM_ROTATION_TO_MECHANISM_DEGREES);
    pivotMotor.setVelocityFactorExternalEncoder(PivotConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_DEG_PER_SECOND);
    pivotMotor.configureTrapezoid(PivotConstants.tunning_values_pivot.MAX_ACCELERATION,
        PivotConstants.tunning_values_pivot.MAX_VELOCITY);
    pivotMotor.configurePIDF(
        PivotConstants.tunning_values_pivot.PID.P,
        PivotConstants.tunning_values_pivot.PID.I,
        PivotConstants.tunning_values_pivot.PID.D,
        0,
        ElevatorConstants.tunning_values_elevator.PID.IZone);
    pivotMotor.burnFlash();
    pivotMotor.setPosition(0);
    pivotMotor.setPositionExternalEncoder(0);
  }

  private void setConfigsEndEffector() {
    endEffectorMotor.setMotorBrake(true);
    endEffectorMotor.setInverted(true);
    endEffectorMotor.setVelocityFactor(EndEffectorConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM);
    endEffectorMotor.burnFlash();
  }

  @Override
  public void periodic() {
    if (!manualControl) {
      setScorerStructureGoals();
    }
    correctPivotPosition();
    SmartDashboard.putNumber("Pivot Position ext", pivotMotor.getPositionExternalEncoder());
    SmartDashboard.putNumber("Pivot Position ", pivotMotor.getPosition());
    SmartDashboard.putNumber("Elevator Position", elevatorMotorLeader.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", elevatorMotorLeader.getVelocity());
    SmartDashboard.putNumber("EndEffector Velocity", endEffectorMotor.getVelocity());
    SmartDashboard.putString("state", state);
    elevatorMotorLeader.updateLogs();
    elevatorMotorFollower.updateLogs();
    pivotMotor.updateLogs();
    endEffectorMotor.updateLogs();
    hasCoralLog.append(this.hasCoral);
    hasAcceleratedLog.append(this.endEffectorAccelerated);
  }

  private void setScorerStructureGoals() {
    if (goalElevator > elevatorMotorLeader.getPosition()) {
      if (pivotSecureForElevator() && !isPivotInternalEncoderLost()) {
        elevatorMotorLeader.setPositionReference(limitGoalElevator(goalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        setPivotTargetPosition(goalPivot);
        pivotStoppedByElevatorLimit.append(false);
        elevatorStoppedByPivotLimit.append(false);
      } else {
        pivotStoppedByElevatorLimit.append(false);
        ;
        elevatorStoppedByPivotLimit.append(true);
        elevatorMotorLeader.setPositionReference(elevatorMotorLeader.getPosition(),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        setPivotTargetPosition(goalPivot);
      }
    } else {
      if (!elevatorSecureForPivot()
          && goalPivot < PivotConstants.tunning_values_pivot.setpoints.UNSECURE_POSITON_FOR_ROTATION_WITH_ELEVATOR_UP
          && isPivotInternalEncoderLost()) {
        elevatorMotorLeader.setPositionReference(limitGoalElevator(goalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        setPivotTargetPosition(pivotMotor.getPosition());
        elevatorStoppedByPivotLimit.append(false);
        pivotStoppedByElevatorLimit.append(true);
      } else {
        elevatorStoppedByPivotLimit.append(false);
        elevatorMotorLeader.setPositionReference(limitGoalElevator(goalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        setPivotTargetPosition(goalPivot);
        pivotStoppedByElevatorLimit.append(false);
      }
    }
  }

  void correctPivotPosition() {
    if (motorNotMoving
        .isStableInCondition(() -> Util.inRange(pivotMotor.getVelocityExternalEncoder(),
            PivotConstants.tunning_values_pivot.MIN_DEAD_BAND_FOR_MOTOR_STOP,
            PivotConstants.tunning_values_pivot.MAX_DEAD_BAND_FOR_MOTOR_STOP))
        && isPivotInternalEncoderLost()) {
      correctingPivot.append(true);
      resetPivotEncoder();
    }
    correctingPivot.append(false);
  }

  private boolean isPivotInternalEncoderLost() {
    return Math.abs(pivotMotor.getPosition() - pivotMotor.getPositionExternalEncoder()) > 5;
  }

  void setPivotTargetPosition(double goal) {
    pivotMotor.setPositionReferenceTrapezoid(0.02, limitGoalPivot(goalPivot), 0,
        PivotConstants.tunning_values_pivot.PID.arbFF);
  }

  public boolean isRobotAbleToScore() {
    return false;
  }

  private void runCoralIntakeDetection() {
    if (this.endEffectorMotor.getVelocity() >= 3000) {
      this.endEffectorAccelerated = true;
    }
    if (endEffectorMotor
        .getVelocity() < EndEffectorConstants.tunning_values_endeffector.VELOCITY_FALL_FOR_INTAKE_DETECTION
        && endEffectorAccelerated) {
      hasCoral = true;
      endEffectorAccelerated = false;
    }
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void intakeFromHP() {
    runCoralIntakeDetection();
    if (!hasCoral) {
      endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE);
    } else {
      endEffectorMotor.set(0);
    }
    goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.COLLECT_HEIGHT;
    goalPivot = PivotConstants.tunning_values_pivot.setpoints.COLLECT_ANGLE;
    state = "INTAKING_FROM_HP";
  }

  @Override
  public void stopIntakeFromHP() {
    endEffectorMotor.set(0);
    state = "START";
    this.endEffectorAccelerated = false;
  }

  @Override
  public void prepareToPlaceCoralOnBranch() {
    assignSetpointsForLevel(this.targetReefHeight);
    state = "PREPARE_TO_PLACE_CORAL";
    branchHeightTarget = this.targetReefHeight.name();
  }

  private void assignSetpointsForLevel(ReefHeight level) {
    switch (level) {
      case L1:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L1_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L1_ANGLE;
        break;
      case L2:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L2_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L2_ANGLE;
        break;
      case L3:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L3_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L3_ANGLE;
        break;
      case L4:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L4_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L4_ANGLE;
        break;
      default:
        break;
    }
  }

  @Override
  public void removeAlgaeFromBranch() {
    endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE);
    assignAlgaeRemovalSetpointsForFace();
    state = "REMOVING_ALGAE_FROM_REEF";
  }

  private int getReefFaceIndexFromPose(Pose3d pose) {
    double minDistance = Double.MAX_VALUE;
    int bestIndex = -1;
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      var facePose = FieldConstants.Reef.centerFaces[i];
      double dx = pose.getX() - facePose.getTranslation().getX();
      double dy = pose.getY() - facePose.getTranslation().getY();
      double distance = Math.hypot(dx, dy);
      if (distance < minDistance) {
        minDistance = distance;
        bestIndex = i;
      }
    }
    return bestIndex;
  }

  private void assignAlgaeRemovalSetpointsForFace() {
    switch (targetAlgaeHeight) {
      case LOW:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.ALGAE_REMOVAL_LOW;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.ALGAE_LOW_REMOVAL;
        break;
      case MID:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.ALGAE_REMOVAL_MID;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.ALGAE_MID_REMOVAL;
        break;
      default:
        break;
    }
  }

  @Override
  public void moveScorerToDefaultPosition() {
    endEffectorMotor.set(0);
    endEffectorAccelerated = false;
    goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
    goalPivot = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE;
    state = "DEFAULT";
  }

  @Override
  public void homeElevator() {
    if (!elevatorHasHomed) {
      elevatorMotorLeader.set(0.1);
      this.state = "HOMING_ELEVATOR";
    } else if (Math.abs(elevatorMotorLeader.getVelocity()) < 0.1) {
      this.state = "ELEVATOR_HOMED";
      elevatorMotorLeader.set(0);
      elevatorMotorLeader.setPosition(ElevatorConstants.ZERO_POSITION_IN_METERS_FROM_GROUND);
      elevatorHasHomed = true;
    }
  }

  @Override
  public void placeCoral() {
    if (targetReefHeight == ReefHeight.L1) {
      endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_EXPELL_L1);
    } else {
      endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_EXPELL);
    }
    this.hasCoral = false;
    this.elevatorHasHomed = false;
    this.state = "PLACING_CORAL";
  }

  @Override
  public boolean isSecuredToPlaceCoral() {
    return isPivotAndElevatorAtSetpoint();
  }

  @Override
  public boolean hasPlaced() {
    return !hasCoral;
  }

  private double limitGoalElevator(double goal) {
    if (goal >= ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT) {
      return ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT;
    } else if (goal <= ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT) {
      return ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
    } else {
      return goal;
    }
  }

  private double limitGoalPivot(double goal) {
    if (goal >= PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE;
    } else if (goal <= PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE;
    } else {
      return goal;
    }
  }

  @Override
  public void setElevatorTestPosition(double testPosition) {
    elevatorMotorLeader.setPositionReference(limitGoalElevator(testPosition),
        ElevatorConstants.tunning_values_elevator.PID.arbFF);
    this.state = "ELEVATOR_TEST_POSITION_" + testPosition;
  }

  @Override
  public void setPivotTestPosition(double testPosition) {
    pivotMotor.setPositionReference(limitGoalPivot(testPosition));
    this.state = "PIVOT_TEST_POSITION_" + testPosition;
  }

  private boolean pivotSecureForElevator() {
    return this.pivotMotor.getPosition() > PivotConstants.tunning_values_pivot.setpoints.SECURE_FOR_ELEVATOR_UP;
  }

  private boolean elevatorSecureForPivot() {
    return this.elevatorMotorLeader
        .getPosition() < ElevatorConstants.tunning_values_elevator.setpoints.SECURE_FOR_PIVOT_ROTATION;
  }

  public void setTargetReefHeight(ReefHeight targetReefHeight) {
    this.targetReefHeight = targetReefHeight;
  }

  @Override
  public void setCoastScorer() {
    this.elevatorMotorLeader.setMotorBrake(false);
    this.elevatorMotorFollower.setMotorBrake(false);
    this.pivotMotor.setMotorBrake(false);
  }

  @Override
  public void setBrakeScorer() {
    this.elevatorMotorLeader.setMotorBrake(true);
    this.elevatorMotorFollower.setMotorBrake(true);
    this.pivotMotor.setMotorBrake(true);
  }

  @Override
  public void setElevatorDutyCycle(double dutyCycle) {
    state = "MANUAL_DUTY_CYCLE_ELEVATOR";
    if (this.elevatorMotorLeader.getPosition() <= ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT
        && dutyCycle > 0) {
      elevatorMotorLeader.set(dutyCycle);
    } else if (elevatorMotorLeader.getPosition() >= ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT
        && dutyCycle < 0) {
      elevatorMotorLeader.set(dutyCycle);
    } else {
      elevatorMotorLeader.set(0);
    }
  }

  @Override
  public void setPivotDutyCycle(double dutyCycle) {
    state = "MANUAL_DUTY_CYCLE_PIVOT";
    if (pivotMotor.getPosition() >= PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE
        && dutyCycle > 0) {
      pivotMotor.set(dutyCycle);
    } else if (pivotMotor.getPosition() <= PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE
        && dutyCycle < 0) {
      pivotMotor.set(dutyCycle);
    } else {
      pivotMotor.set(0);
    }
  }

  @Override
  public void setEndEffectorDutyCycle(double dutyCycle) {
    manualControl = true;
    endEffectorMotor.set(dutyCycle);
  }

  @Override
  public boolean isAtCollectPosition() {
    return Util.atSetpoint(this.elevatorMotorLeader.getPosition(),
        ElevatorConstants.tunning_values_elevator.setpoints.COLLECT_HEIGHT, 0.05)
        && Util.atSetpoint(this.pivotMotor.getPosition(), PivotConstants.tunning_values_pivot.setpoints.COLLECT_ANGLE,
            2);
  }

  @Override
  public boolean isAtDefaultPosition() {
    return Util.atSetpoint(this.elevatorMotorLeader.getPosition(),
        ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT, 0.05)
        && Util.atSetpoint(this.pivotMotor.getPosition(), PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE,
            2);
  }

  private void resetPivotEncoder() {
    pivotMotor.setPosition(pivotMotor.getPositionExternalEncoder());
  }

  @Override
  public boolean isAtRemovePosition() {
    return isPivotAndElevatorAtSetpoint();
  }

  private boolean isPivotAndElevatorAtSetpoint() {
    return Util.atSetpoint(this.elevatorMotorLeader.getPosition(), this.goalElevator, 0.05)
        && Util.atSetpoint(this.pivotMotor.getPosition(), this.goalPivot, 2);
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    this.elevatorMotorLeader.set(Volts.of(voltage));
  }

  @Override
  public void setTargetBranchLevel(ReefHeight reefHeight) {
    this.targetReefHeight = reefHeight;
  }

  @Override
  public void setTargetAlgaeHeight(AlgaeHeight algaeHeight) {
    this.targetAlgaeHeight = algaeHeight;
  }
}
