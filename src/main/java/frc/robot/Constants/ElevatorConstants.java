package frc.robot.constants;

public class ElevatorConstants {
  public static final int ID_elevatorLeaderMotor = 14;
  public static final int ID_elevatorFollowerMotor = 16;
  public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS = 0;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND = 0;

  public class tunning_values_elevator {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double POSITION_ERROR_ALLOWED = 0;

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double arbFF = 0;// works as a kG
      public static final double IZone = 0;
    }

    public class setpoints {
      public static final double MAX_HEIGHT = 2.5;
      public static final double MIN_HEIGHT = 0;

      public static final double L1_HEIGHT = 0.5;
      public static final double L2_HEIGHT = 1.0;
      public static final double L3_HEIGHT = 1.5;
      public static final double L4_HEIGHT = 2.0;
    }
  }
}
