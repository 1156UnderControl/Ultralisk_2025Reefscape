package frc.robot.subsystems.Intake;

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
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'intake'");
  }

  @Override
  public void expell() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'expell'");
  }

  @Override
  public void goToIntakePosition() {
  }

  @Override
  public void goToSecuredPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'goToSecuredPosition'");
  }

  @Override
  public boolean isAtSetPoint() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isAtSetPoint'");
  }

}
