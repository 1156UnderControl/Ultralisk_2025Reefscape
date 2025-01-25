package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private IMotor intakeMotor;

  public IntakeSubsystem() {
    this.intakeMotor = new SparkMAXMotor(0);
  }

  @Override
  public void intake() {
  }

  @Override
  public void expell() {
  }

  @Override
  public void goToIntakePosition() {
  }

  @Override
  public void goToSecuredPosition() {
  }

  @Override
  public boolean isAtSetPoint() {
    return false;
  }
}
