package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScorerConstants;
import frc.robot.joysticks.ControlBoard;

public class ScorerSubsystem extends SubsystemBase implements IScorer {

  private static ScorerSubsystem instance;
  private IMotor elevatorMotorLeader = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, "ELEVATOR_MASTER");
  private IMotor elevatorMotorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor,
      "ELEVATOR_FOLLOWER");

  private final IMotor pivotMotor = new SparkMAXMotor(ScorerConstants.ID_pivotMotor, true, "PIVOT");
  private final IMotor endEffectorMotor = new SparkMAXMotor(ScorerConstants.ID_endEffectorMotor, "END_EFFECTOR");
  private boolean hasCoral = false;
  private ControlBoard controlBoard = ControlBoard.getInstance();
  private boolean setCoastScorer;
  private boolean setBrakeScorer;

  public static ScorerSubsystem getInstance() {
    if (instance == null) {
      instance = new ScorerSubsystem();
    }
    return instance;
  }

  private ScorerSubsystem() {
    setConfigsElevator();
  }

  int Previousvelocity;
  private Object scorer;

  private void setConfigsElevator() {
    elevatorMotorLeader.setMotorBrake(true);
    elevatorMotorFollower.setMotorBrake(true);
    elevatorMotorLeader.setLoopRampRate(0.5);
    elevatorMotorFollower.setLoopRampRate(0.5);
    elevatorMotorFollower.setFollower(ElevatorConstants.ID_elevatorLeaderMotor, true);
    elevatorMotorLeader.setPositionFactor(ElevatorConstants.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS);
    elevatorMotorLeader.configureMotionProfiling(
        ElevatorConstants.tunning_values_elevator.PID.P,
        ElevatorConstants.tunning_values_elevator.PID.I,
        ElevatorConstants.tunning_values_elevator.PID.D,
        ElevatorConstants.tunning_values_elevator.PID.F,
        ElevatorConstants.tunning_values_elevator.MAX_VELOCITY,
        ElevatorConstants.tunning_values_elevator.MAX_ACCELERATION,
        ElevatorConstants.tunning_values_elevator.POSITION_ERROR_ALLOWED);
    elevatorMotorFollower.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Subsystem Scorer", ScorerSubsystem.getInstance());
  }

  public void setCoastToRobot() {
    this.setCoastScorer = true;
  }

  public void setBrakeToRobot() {
    this.setBrakeScorer = true;
  }

  public boolean isRobotAbleToScore() {
    return false;
  }

  public void detectthecollect() {
    if (endEffectorMotor.getVelocity() < Previousvelocity - 50) {
      hasCoral = true;
    } else {
      hasCoral = false;
    }
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void intake() {

  }

  @Override
  public void intakeFromHP() {
  }

  @Override
  public void prepareToPlaceL1() {
  }

  @Override
  public void prepareToPlaceL2() {
  }

  @Override
  public void prepareToPlaceL3() {
  }

  @Override
  public void prepareToPlaceL4() {
  }

  @Override
  public void place() {
  }

  @Override
  public void expell() {
  }

  @Override
  public boolean isSecuredToPlace() {
    return false;
  }

  @Override
  public boolean hasPlaced() {
    return false;
  }

  @Override
  public void stopIntaking() {
  }

  @Override
  public boolean isAtSetPoint() {
    return false;
  }
}
