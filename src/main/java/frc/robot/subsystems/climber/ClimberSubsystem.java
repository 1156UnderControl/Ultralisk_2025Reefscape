package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.Servo;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem implements IClimber {
  private static ClimberSubsystem instance;
  private IMotor cageIntakeMotor = new SparkMAXMotor(ClimberConstants.ID_cageIntakeMotor, "Climber Intake Motor");
  private IMotor climberArmMotor = new TalonFXMotor(ClimberConstants.ID_climberArmMotor, GravityTypeValue.Arm_Cosine,
      "Climber Arm Motor");
  private Servo climberServoMotor = new Servo(0);

  private double previousVelocity = 0;
  private boolean isCageCollected = false;

  boolean cageIntakeAccelerated = false;

  double armGoal = ClimberConstants.tunning_values_arm.setpoints.STOW_ANGLE;

  private String state = "START";

  private boolean stopClimber = false;

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  private ClimberSubsystem() {
    configureClimberMotor();
  }

  private void configureClimberMotor() {
    climberArmMotor.setInverted(false);
    climberArmMotor.setPositionFactor(250);
    climberArmMotor.configurePIDF(
        ClimberConstants.tunning_values_arm.PID.P,
        ClimberConstants.tunning_values_arm.PID.I,
        ClimberConstants.tunning_values_arm.PID.D, 0);
    cageIntakeMotor.burnFlash();
    climberArmMotor.setMotorBrake(true);
    climberArmMotor.setPosition(0);
  }

  @Override
  public void climb() {
    stopClimber = false;
    this.armGoal = ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE;
  }

  public void moveArmToPosition(double position, double arbFF) {
    double goal = limitGoalArm(position);
    if (stopClimber) {
      climberArmMotor.set(0);
      return;
    }
    climberArmMotor.setPositionReference(goal);
  }

  private double limitGoalArm(double goal) {
    if (goal >= ClimberConstants.tunning_values_arm.setpoints.MAX_ANGLE) {
      return ClimberConstants.tunning_values_arm.setpoints.MAX_ANGLE;
    } else if (goal <= ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE) {
      return ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE;
    } else {
      return goal;
    }
  }

  public void periodic() {
    moveArmToPosition(this.armGoal, 0);
    climberArmMotor.updateLogs();
  }

  @Override
  public boolean isAtIntakeCagePosition() {
    return Util.atSetpoint(this.climberArmMotor.getPosition(),
        ClimberConstants.tunning_values_arm.setpoints.INTAKE_CAGE_ANGLE, 0.01);
  }

  @Override
  public boolean isAtClimbPosition() {
    return Util.atSetpoint(this.climberArmMotor.getPosition(),
        ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE, 0.01);
  }

  @Override
  public void goToIntakeCagePosition() {
    stopClimber = false;
    this.armGoal = ClimberConstants.tunning_values_arm.setpoints.INTAKE_CAGE_ANGLE;
  }

  @Override
  public void intakeCage() {
    cageIntakeMotor.set(ClimberConstants.tunning_values_intake.setpoints.DUTY_CYCLE_INTAKE);
    state = "INTAKING_CAGE";
  }

  @Override
  public boolean isCageCollected() {
    return isCageCollected;
  }

  @Override
  public void stopIntakingCage() {
    cageIntakeMotor.set(0);
  }

  @Override
  public void setCoastClimber() {
    cageIntakeMotor.setMotorBrake(false);
    climberArmMotor.setMotorBrake(false);
  }

  @Override
  public void setBrakeClimber() {
    cageIntakeMotor.setMotorBrake(true);
    climberArmMotor.setMotorBrake(true);
  }

  @Override
  public void lockClimber() {
    climberServoMotor.set(1);
  }

  @Override
  public void unlockClimber() {
    climberServoMotor.set(0);
  }

  @Override
  public void stopClimberArm() {
    stopClimber = true;
  }

  @Override
  public void goToStowPosition() {
    stopClimber = false;
    this.armGoal = ClimberConstants.tunning_values_arm.setpoints.STOW_ANGLE;
  }
}
