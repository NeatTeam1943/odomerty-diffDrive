package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final Measure<Velocity<Distance>> kDriveSpeed = Units.MetersPerSecond.of(3);
    public static final Measure<Velocity<Angle>> kTurnSpeed = Units.RotationsPerSecond.of(2 * Math.PI);
    public static final Measure<Distance> kTrackWidth = Units.Inches.of(15);
    public static final Measure<Distance> kWheelDiameter = Units.Inches.of(6);

    public static final double kFalconCPR = 2048;
    public static final double kChassisGR = 10.71;

    public static final int kLeftMasterId = 1;
    public static final int kLeftFollowerId = 2;
    public static final int kRightMasterId = 3;
    public static final int kRightFollowerId = 4;

    public static final int kImuId = 5;

    public static final double kLeftP = 0.1;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0;

    public static final double kRightP = 0.1;
    public static final double kRightI = 0;
    public static final double kRightD = 0;

    public static final Measure<Voltage> kV = Units.Volts.of(12);
    public static final Measure<Per<Voltage, Velocity<Distance>>> kS = Units.VoltsPerMeterPerSecond.of(3);
  }
}
