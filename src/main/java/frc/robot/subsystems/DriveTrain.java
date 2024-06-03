package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private final TalonFX m_leftFollower;
  private final TalonFX m_leftMaster;
  private final TalonFX m_rightFollower;
  private final TalonFX m_rightMaster;

  private final MutableMeasure<Distance> m_leftPosition;
  private final MutableMeasure<Distance> m_rightPosition;

  private final Pigeon2 m_imu;

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;

  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;

  private final SimpleMotorFeedforward m_feedforward;

  public DriveTrain() {
    m_leftMaster = new TalonFX(DriveTrainConstants.kLeftMasterId);
    m_leftFollower = new TalonFX(DriveTrainConstants.kLeftFollowerId);
    m_rightMaster = new TalonFX(DriveTrainConstants.kRightMasterId);
    m_rightFollower = new TalonFX(DriveTrainConstants.kRightFollowerId);

    m_imu = new Pigeon2(DriveTrainConstants.kImuId);

    m_leftPIDController = new  PIDController(DriveTrainConstants.kLeftP, DriveTrainConstants.kLeftI, DriveTrainConstants.kLeftD);
    m_rightPIDController = new PIDController(DriveTrainConstants.kRightP, DriveTrainConstants.kRightI, DriveTrainConstants.kRightD);
    m_feedforward = new SimpleMotorFeedforward(DriveTrainConstants.kS.magnitude(), DriveTrainConstants.kV.magnitude());

    m_leftPosition = MutableMeasure.zero(Units.Meters);
    m_rightPosition = MutableMeasure.zero(Units.Meters);

    m_kinematics = new DifferentialDriveKinematics(DriveTrainConstants.kTrackWidth);

    setupMotorFollowrs();
  }

  private void setupMotorFollowrs() {
    m_leftFollower.setControl(new Follower(m_leftMaster.getDeviceID(), false));
    m_rightFollower.setControl(new Follower(m_rightMaster.getDeviceID(), false));

    m_rightFollower.setInverted(true);
  }

  private void updateEncoderPosition() {
    double leftPosAvg = (m_leftMaster.getPosition().getValueAsDouble() + m_leftFollower.getPosition().getValueAsDouble()) / 2;
    double rightPosAvg = (m_rightMaster.getPosition().getValueAsDouble() + m_rightFollower.getPosition().getValueAsDouble()) / 2;

    m_leftPosition.mut_replace(Units.Meter.of(nativeUnitsToDistanceMeters(leftPosAvg))); 
    m_rightPosition.mut_replace(Units.Meter.of(nativeUnitsToDistanceMeters(rightPosAvg)));
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / DriveTrainConstants.kFalconCPR;
    double wheelRotations = motorRotations / DriveTrainConstants.kChassisGR;
    var position = DriveTrainConstants.kWheelDiameter.times(Math.PI).times(wheelRotations);

    return position.in(Units.Meters);
  }

  @Override
  public void periodic() {
    updateEncoderPosition();
  }
}