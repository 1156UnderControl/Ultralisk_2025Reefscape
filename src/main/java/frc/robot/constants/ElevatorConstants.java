package frc.robot.constants;

public class ElevatorConstants {
  public static final int ID_elevatorLeaderMotor = 6;
  public static final int ID_elevatorFollowerMotor = 7;
  public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS = 0.024437;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND = 0.0004072;
  public static final double ZERO_POSITION_IN_METERS_FROM_GROUND = 0.38;
  public static final double PASSIVE_HOMING_RANGE = 0.02;

  public class tunning_values_elevator {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double MAX_VELOCITY = 2000;
    public static final double MAX_ACCELERATION = 10000;
    public static final double POSITION_ERROR_ALLOWED = 0;

    public class PID {
      public static final double P = 4;
      public static final double I = 0.004;
      public static final double D = 0;
      public static final double arbFF = 0.025;// ks as a kG
      public static final double IZone = 0.2;
    }

    public class setpoints {
      public static final double MAX_HEIGHT = 2.35;
      public static final double MIN_HEIGHT = ZERO_POSITION_IN_METERS_FROM_GROUND;
      public static final double NET_HEIGHT = 1.1;
      public static final double PROCESSOR_HEIGHT = 0.35;
      public static final double L1_HEIGHT = 0.44;
      public static final double L2_HEIGHT = 0.8;
      public static final double L3_HEIGHT = 1.19;
      public static final double L4_HEIGHT = 2.12;
      public static final double ALGAE_COLLECT_MID = 1.37;
      public static final double ALGAE_COLLECT_LOW = 0.98;
      public static final double SECURE_FOR_PIVOT_ROTATION = 1;
      public static final double COLLECT_HEIGHT = MIN_HEIGHT;
      public static final double COLLECT_ALGAE_REEF_MID = 1.14;
      public static final double COLLECT_ALGAE_REEF_LOW = 0.82;
      public static final double COLLECT_ALGAE_GROUND = ZERO_POSITION_IN_METERS_FROM_GROUND;

    }

    public class stable_transition {
      public static final double DISTANCE_FOR_FULL_DEPLOYMENT = 0.3;
      public static final double DISTANCE_FOR_DEPLOYMENT_START = 1.0;
      public static final double SAFE_CRUISE_HEIGHT = 1.1;
    }
  }
}
