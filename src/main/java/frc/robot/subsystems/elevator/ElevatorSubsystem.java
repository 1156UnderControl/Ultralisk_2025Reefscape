package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase implements IElevator {
  private IMotor elevatorMotor1 = new SparkMAXMotor(ElevatorConstants.ID_elevatorMotor1);
  private IMotor elevatorMotor2 = new SparkMAXMotor(ElevatorConstants.ID_elevatorMotor2);

  public ElevatorSubsystem() {

  }

}
