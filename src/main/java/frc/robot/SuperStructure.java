package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IArm;

public class SuperStructure {
    public IArm arm;

    private SubsystemBase armSubsystem;

    private ControlBoard controlBoard = ControlBoard.getInstance();
}
