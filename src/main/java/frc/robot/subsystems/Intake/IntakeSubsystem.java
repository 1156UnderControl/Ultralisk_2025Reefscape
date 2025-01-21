package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private IMotor intakeMotor;

  public IntakeSubsystem() {
    this.intakeMotor = new SparkMAXMotor(0);
  }

}
