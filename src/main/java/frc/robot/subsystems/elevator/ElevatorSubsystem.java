package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;

public class ElevatorSubsystem extends SubsystemBase implements IElevator {
  private IMotor elevatorMotor1;
  private IMotor elevatorMotor2;

  public ElevatorSubsystem() {
    this.elevatorMotor1 = new SparkMAXMotor(0);
    this.elevatorMotor2 = new SparkMAXMotor(0);
  }

}
