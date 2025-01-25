package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final Pose2d intakeRollerPositionRelativeToRobotCenter = new Pose2d(0.46, 0, new Rotation2d());
  public static final int ID_intake = 0;
  public static final int port_IR = 0;

  public class TunningValues {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double F = 0;

    }

    public class setpoints {

      public static final double SPEED_INTAKE = 0;
      public static final double SPEED_EXPELL = 0;

      public static final double LIMIT_MAX = 0;
      public static final double LIMIT_MIN = 0;

      public static final double ANGLE_SECURED = 0;
      public static final double ANGLE_INTAKE = 0;
    }

  }
}
