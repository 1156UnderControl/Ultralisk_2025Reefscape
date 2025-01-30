package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScorerConstants;

public class ScorerSubsystem extends SubsystemBase implements IScorer {

  private IMotor elevatorMotorLeader = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, "ELEVATOR_MASTER");
  private IMotor elevatorMotorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor,
      "ELEVATOR_FOLLOWER");

  private final IMotor pivotMotor = new SparkMAXMotor(ScorerConstants.ID_pivotMotor, true, "PIVOT");
  private final IMotor endEffectorMotor = new SparkMAXMotor(ScorerConstants.ID_endEffectorMotor, "END_EFFECTOR");
  private boolean hascollected = false;

  public ScorerSubsystem() {
    setConfigsElevator();
  }

  int Previousvelocity;

  private void setConfigsElevator() {
    elevatorMotorLeader.setMotorBrake(true);
    elevatorMotorFollower.setMotorBrake(true);
    elevatorMotorLeader.setLoopRampRate(0.5);
    elevatorMotorFollower.setLoopRampRate(0.5);
    elevatorMotorFollower.setFollower(ElevatorConstants.ID_elevatorLeaderMotor, true);
    elevatorMotorLeader.setPositionFactor(ElevatorConstants.POSITION_FACTOR_MOTOR_ROT_TO_METERS);
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
  }

  public void detectthecollect() {
    if (endEffectorMotor.getVelocity() < Previousvelocity - 50) {
      hascollected = true;
    } else {
      hascollected = false;
    }
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
