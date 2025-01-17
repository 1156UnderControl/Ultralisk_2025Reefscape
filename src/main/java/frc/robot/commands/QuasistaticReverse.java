package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class QuasistaticReverse extends Command {
    private ArmSubsystem robot;
  
    public QuasistaticReverse(ArmSubsystem robot) {
      this.robot = robot;
      this.addRequirements(robot);
    }
  
    @Override
    public void execute() {
      this.robot.sysIdQuasistaticReverse();
    }
  
    @Override
    public boolean isFinished() {
      return this.robot.isAtSetPoint();
    }
}