package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Sensors.InfraRed;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.FieldConstants.AlgaeHeight;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.PivotConstants;

public class ScorerSubsystem implements IScorer {

  private static ScorerSubsystem instance;
  private IMotor elevatorMotorLeader = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, "ELEVATOR_MASTER");
  private IMotor elevatorMotorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor,
      "ELEVATOR_FOLLOWER");

  private final IMotor pivotMotor = new SparkFlexMotor(PivotConstants.ID_pivotMotor, false, "PIVOT");
  private final IMotor endEffectorMotor = new SparkMAXMotor(EndEffectorConstants.ID_endEffectorMotor, "END_EFFECTOR");
  private final InfraRed coralInfraRedSensor = new InfraRed(EndEffectorConstants.Port_coralInfraRed, false);

  private boolean hasCoral = true;
  private boolean elevatorHasHomed = false;
  private double goalElevator = ElevatorConstants.ZERO_POSITION_IN_METERS_FROM_GROUND;
  private double goalPivot = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE;

  StabilizeChecker motorNotMoving = new StabilizeChecker(0.2);

  StabilizeChecker pivotInternalEncoderIsLost = new StabilizeChecker(0.2);

  CustomBooleanLogger correctingPivot = new CustomBooleanLogger("/ScorerSubsystem/Correcting Pivot");

  private String state = "START";

  private String branchHeightTarget = "NONE";

  CustomStringLogger scorerStateLogger = new CustomStringLogger("/ScorerSubsystem/State");

  CustomStringLogger targetAlgaeLevelLogger = new CustomStringLogger("/ScorerSubsystem/targetAlgaeLevel");

  CustomStringLogger targetBranchLevelLogger = new CustomStringLogger("/ScorerSubsystem/targetBranchLevel");

  CustomBooleanLogger hasCoralLog = new CustomBooleanLogger("/ScorerSubsystem/hasCoral");

  CustomBooleanLogger hasAcceleratedLog = new CustomBooleanLogger("/ScorerSubsystem/hasAccelerated");

  CustomStringLogger targetReefLevelLog = new CustomStringLogger("/ScorerSubsystem/targetReefLevel");

  CustomBooleanLogger elevatorStoppedByPivotLimit = new CustomBooleanLogger(
      "/ScorerSubsystem/elevatorStoppedByPivotLimit");

  CustomBooleanLogger pivotStoppedByElevatorLimit = new CustomBooleanLogger(
      "/ScorerSubsystem/pivotStoppedByElevatorLimit");

  private ReefLevel targetReefLevel = ReefLevel.L1;

  private AlgaeHeight targetAlgaeHeight = AlgaeHeight.LOW;

  private boolean manualControl = false;

  private boolean endEffectorAccelerated = false;

  private double lastGoalPivot = goalPivot;

  private StabilizeChecker pivotAndElevatorStableInPosition = new StabilizeChecker(0.15);

  private Supplier<Double> distanceToTargetPoseProvider = () -> this.goStraightToTargetHeightProvider();

  private StabilizeChecker stableSensors = new StabilizeChecker(0.2);

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
    pivotMotor.setPosition(pivotMotor.getPositionExternalAbsoluteEncoder());
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
    updateLogs();
  }

  private void updateLogs() {
    elevatorMotorLeader.updateLogs();
    elevatorMotorFollower.updateLogs();
    pivotMotor.updateLogs();
    endEffectorMotor.updateLogs();
    hasCoralLog.append(this.hasCoral);
    hasAcceleratedLog.append(this.endEffectorAccelerated);
    scorerStateLogger.append(this.state);
    targetBranchLevelLogger.append(this.targetReefLevel.name());
    targetAlgaeLevelLogger.append(this.targetAlgaeHeight.name());
    targetReefLevelLog.append(this.targetReefLevel.name());
    SmartDashboard.putString("ScorerSubsystem/TargetLevelName", this.targetReefLevel.name());
    SmartDashboard.putString("ScorerSubsystem/TargetReefBranch", branchHeightTarget);
    SmartDashboard.putBoolean("ScorerSubsystem/HasCoral", this.hasCoral());
  }

  private void setScorerStructureGoals() {
    double stabilizedGoalElevator = this.limitTargetToStableHeight(goalElevator);
    if (stabilizedGoalElevator > elevatorMotorLeader.getPosition()) {
      if (pivotSecureForElevator()) {
        elevatorMotorLeader.setPositionReference(limitGoalElevator(stabilizedGoalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        setPivotTargetPosition();
        pivotStoppedByElevatorLimit.append(false);
        elevatorStoppedByPivotLimit.append(false);
      } else {
        pivotStoppedByElevatorLimit.append(false);
        elevatorStoppedByPivotLimit.append(true);
        elevatorMotorLeader.set(0);
        setPivotTargetPosition();
      }
    } else {
      if (!elevatorSecureForPivot()
          && goalPivot < PivotConstants.tunning_values_pivot.setpoints.UNSECURE_POSITON_FOR_ROTATION_WITH_ELEVATOR_UP) {
        elevatorMotorLeader.setPositionReference(limitGoalElevator(stabilizedGoalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        pivotMotor.set(0);
        elevatorStoppedByPivotLimit.append(false);
        pivotStoppedByElevatorLimit.append(true);
      } else {
        elevatorStoppedByPivotLimit.append(false);
        elevatorMotorLeader.setPositionReference(limitGoalElevator(stabilizedGoalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        setPivotTargetPosition();
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
    return pivotInternalEncoderIsLost.isStableInCondition(
        () -> Math.abs(pivotMotor.getPosition() - pivotMotor.getPositionExternalAbsoluteEncoder()) > 5);
  }

  void setPivotTargetPosition() {
    pivotMotor.setPositionReferenceTrapezoid(0.02, limitGoalPivot(goalPivot), 0,
        PivotConstants.tunning_values_pivot.PID.arbFF);
  }

  public boolean isRobotAbleToScore() {
    return false;
  }

  private void runCoralIntakeDetection() {
    if (this.endEffectorMotor.getVelocity() >= 2500) {
      this.endEffectorAccelerated = true;
    }
    if ((endEffectorMotor
        .getVelocity() < EndEffectorConstants.tunning_values_endeffector.VELOCITY_FALL_FOR_INTAKE_DETECTION
        && endEffectorAccelerated)
        && stableSensors.isStableInCondition(() -> this.isAtCollectPosition() && coralInfraRedSensor.getAsBoolean())) {
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
      goalPivot = PivotConstants.tunning_values_pivot.setpoints.COLLECT_ANGLE;
    } else {

      endEffectorMotor.set(0);
    }
    goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.COLLECT_HEIGHT;
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
    this.distanceToTargetPoseProvider = () -> this.goStraightToTargetHeightProvider();
    assignSetpointsForLevel(this.targetReefLevel);
    state = "PREPARE_TO_PLACE_CORAL";
    branchHeightTarget = this.targetReefLevel.name();
  }

  @Override
  public void prepareToPlaceCoralOnBranch(Supplier<Double> distanceToTargetPoseProvider) {
    this.distanceToTargetPoseProvider = distanceToTargetPoseProvider;
    assignSetpointsForLevel(this.targetReefLevel);
    state = "PREPARE_TO_PLACE_CORAL";
    branchHeightTarget = this.targetReefLevel.name();
    this.runCoralAntiLockRoutine();
  }

  private void assignSetpointsForLevel(ReefLevel level) {
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
      case TO_L4:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L3_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L4_ANGLE;
        break;
      default:
        break;
    }
  }

  @Override
  public void removeAlgaeFromBranch() {
    assignAlgaeRemovalSetpointsForAlgaeHeight();
    state = "MOVING_ELEVATOR_TO_REMOVE_ALGAE_FROM_REEF";
  }

  @Override
  public void removeAlgaeEndEffector() {
    endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE);
    state = "REMOVING_ALGAE_FROM_REEF";
  }

  @Override
  public void stopEndEffector() {
    endEffectorMotor.set(0);
  }

  private void assignAlgaeRemovalSetpointsForAlgaeHeight() {
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
    endEffectorAccelerated = false;
    if (!this.hasCoral) {
      endEffectorMotor.set(0);
      goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.COLLECT_HEIGHT;
      goalPivot = PivotConstants.tunning_values_pivot.setpoints.COLLECT_ANGLE;
      state = "DEFAULT_WITHOUT_CORAL";
      return;
    }
    this.runCoralAntiLockRoutine();
    goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
    goalPivot = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE;
    state = "DEFAULT_WITH_CORAL";
  }

  @Override
  public void placeCoral() {
    if (targetReefLevel == ReefLevel.L1) {
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
    return this.pivotMotor
        .getPositionExternalAbsoluteEncoder() > PivotConstants.tunning_values_pivot.setpoints.SECURE_FOR_ELEVATOR_UP;
  }

  private boolean elevatorSecureForPivot() {
    return this.elevatorMotorLeader
        .getPosition() < ElevatorConstants.tunning_values_elevator.setpoints.SECURE_FOR_PIVOT_ROTATION;
  }

  public void setTargetReefLevel(ReefLevel targetReefHeight) {
    this.targetReefLevel = targetReefHeight;
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
    boolean isAtCollectPosition = Util.atSetpoint(this.elevatorMotorLeader.getPosition(),
        ElevatorConstants.tunning_values_elevator.setpoints.COLLECT_HEIGHT, 0.05)
        && Util.atSetpoint(this.pivotMotor.getPositionExternalAbsoluteEncoder(),
            PivotConstants.tunning_values_pivot.setpoints.COLLECT_ANGLE,
            6);
    return isAtCollectPosition;
  }

  @Override
  public boolean isAtDefaultPosition() {
    return Util.atSetpoint(this.elevatorMotorLeader.getPosition(),
        this.goalElevator, 0.05)
        && Util.atSetpoint(this.pivotMotor.getPosition(), this.goalPivot,
            2);
  }

  private void resetPivotEncoder() {
    pivotMotor.setPosition(pivotMotor.getPositionExternalAbsoluteEncoder());
  }

  @Override
  public boolean isScorerAtPosition() {
    return isPivotAndElevatorAtSetpoint();
  }

  private boolean isPivotAndElevatorAtSetpoint() {
    return this.pivotAndElevatorStableInPosition
        .isStableInCondition(() -> Util.atSetpoint(this.elevatorMotorLeader.getPosition(), this.goalElevator, 0.05)
            && Util.atSetpoint(this.pivotMotor.getPositionExternalAbsoluteEncoder(), this.goalPivot, 5));
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    this.elevatorMotorLeader.set(Volts.of(voltage));
  }

  @Override
  public void setTargetBranchLevel(ReefLevel reefHeight) {
    this.targetReefLevel = reefHeight;
  }

  @Override
  public void setTargetAlgaeHeight(AlgaeHeight algaeHeight) {
    this.targetAlgaeHeight = algaeHeight;
  }

  @Override
  public ReefLevel getTargetReefLevel() {
    return this.targetReefLevel;
  }

  @Override
  public boolean isElevatorInHighPosition() {
    return this.elevatorMotorLeader.getPosition() > 0.9;
  }

  private double goStraightToTargetHeightProvider() {
    return 0;
  }

  private double limitTargetToStableHeight(double targetHeight) {
    if (targetHeight < ElevatorConstants.tunning_values_elevator.stable_transition.SAFE_CRUISE_HEIGHT) {
      return targetHeight;
    }
    double distanceToTargetPose = this.distanceToTargetPoseProvider.get();
    if (distanceToTargetPose < ElevatorConstants.tunning_values_elevator.stable_transition.DISTANCE_FOR_FULL_DEPLOYMENT) {
      return targetHeight;
    }
    if (distanceToTargetPose > ElevatorConstants.tunning_values_elevator.stable_transition.DISTANCE_FOR_DEPLOYMENT_START) {
      return ElevatorConstants.tunning_values_elevator.stable_transition.SAFE_CRUISE_HEIGHT;
    }
    double heightToRaise = targetHeight
        - ElevatorConstants.tunning_values_elevator.stable_transition.SAFE_CRUISE_HEIGHT;
    double distanceNeededToRaise = ElevatorConstants.tunning_values_elevator.stable_transition.DISTANCE_FOR_DEPLOYMENT_START
        - ElevatorConstants.tunning_values_elevator.stable_transition.DISTANCE_FOR_FULL_DEPLOYMENT;
    double currentProgressInDistanceToRaise = 1 - ((distanceToTargetPose
        - ElevatorConstants.tunning_values_elevator.stable_transition.DISTANCE_FOR_FULL_DEPLOYMENT)
        / distanceNeededToRaise);
    double currentTargetHeight = (heightToRaise * currentProgressInDistanceToRaise)
        + ElevatorConstants.tunning_values_elevator.stable_transition.SAFE_CRUISE_HEIGHT;
    return currentTargetHeight;
  }

  public void runCoralAntiLockRoutine() {
    if (this.pivotMotor
        .getPositionExternalAbsoluteEncoder() < (PivotConstants.tunning_values_pivot.setpoints.COLLECT_ANGLE + 10)) {
      endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE);
    } else {
      if (hasCoral) {
        endEffectorMotor.set(0.1);
      } else {
        endEffectorMotor.set(0);
      }
    }
  }

  @Override
  public void setAngle180() {
    this.goalPivot = 180;
  }

  @Override
  public void setAngle10() {
    this.goalPivot = 20;
  }
}
