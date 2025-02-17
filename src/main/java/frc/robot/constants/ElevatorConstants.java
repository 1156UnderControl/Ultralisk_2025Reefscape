package frc.robot.constants;

public class ElevatorConstants {
  public static final int ID_elevatorLeaderMotor = 6;
  public static final int ID_elevatorFollowerMotor = 7;
  public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS = 1;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND = 1;

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
      public static final double MAX_HEIGHT = 65.0;
      public static final double MIN_HEIGHT = 0;
      public static final double L1_HEIGHT = 0.5;
      public static final double L2_HEIGHT = 1.0;
      public static final double L3_HEIGHT = 1.5;
      public static final double L4_HEIGHT = 2.0;
      public static final double FACE0_ALGAE_REMOVAL = 0.4;
      public static final double FACE1_ALGAE_REMOVAL = 0.6;
      public static final double FACE2_ALGAE_REMOVAL = 0.8;
      public static final double FACE3_ALGAE_REMOVAL = 1.0;
      public static final double FACE4_ALGAE_REMOVAL = 1.2;
      public static final double FACE5_ALGAE_REMOVAL = 1.4;
      public static final double SECURE_FOR_PIVOT_ROTATION = 1.5;
      public static final double COLLECT_HEIGHT = MIN_HEIGHT;
    }
  }
}
