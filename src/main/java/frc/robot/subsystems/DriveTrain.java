package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private final TalonFX m_leftFollower;
  private final TalonFX m_leftMaster;
  private final TalonFX m_rightFollower;
  private final TalonFX m_rightMaster;

  private final MutableMeasure<Distance> m_leftPosition;
  private final MutableMeasure<Distance> m_rightPosition;

  private final MutableMeasure<Velocity<Distance>> m_leftVelocity;
  private final MutableMeasure<Velocity<Distance>> m_rightVelocity;

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

    m_leftVelocity = MutableMeasure.zero(Units.MetersPerSecond);
    m_rightVelocity = MutableMeasure.zero(Units.MetersPerSecond);

    m_kinematics = new DifferentialDriveKinematics(DriveTrainConstants.kTrackWidth);

    setupMotorFollowrs();

    m_odometry = new DifferentialDriveOdometry(m_imu.getRotation2d(), m_rightPosition, m_leftPosition);
  }

  private void setupMotorFollowrs() {
    m_leftFollower.setControl(new Follower(m_leftMaster.getDeviceID(), false));
    m_rightFollower.setControl(new Follower(m_rightMaster.getDeviceID(), false));

    m_rightFollower.setInverted(true);
  }

  private void updateEncoders() {
    double leftPosAvg = (m_leftMaster.getPosition().getValueAsDouble() + m_leftFollower.getPosition().getValueAsDouble()) / 2;
    double rightPosAvg = (m_rightMaster.getPosition().getValueAsDouble() + m_rightFollower.getPosition().getValueAsDouble()) / 2;

    double leftPosVelocity = (m_leftMaster.getVelocity().getValueAsDouble() + m_leftFollower.getVelocity().getValueAsDouble()) / 2;
    double rightPosVelocity = (m_rightMaster.getVelocity().getValueAsDouble() + m_rightFollower.getVelocity().getValueAsDouble()) / 2;

    rotationsToDistanceMeters(leftPosAvg, rightPosAvg, leftPosVelocity, rightPosVelocity);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFF = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFF = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(m_leftVelocity.magnitude(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightVelocity.magnitude(), speeds.rightMetersPerSecond);

    m_leftMaster.setVoltage(leftOutput + leftFF);
    m_rightMaster.setVoltage(rightOutput + rightFF);
  }

  public void drive(Measure<Velocity<Distance>> xSpeed, Measure<Velocity<Angle>> rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, Units.MetersPerSecond.zero(), rot));
    setSpeeds(wheelSpeeds);
  }

  private double nativeUnitsToWheelRotations(double sensorCounts){
    double motorRotations = (double)sensorCounts / DriveTrainConstants.kFalconCPR;
    double wheelRotations = motorRotations / DriveTrainConstants.kChassisGR;

    return wheelRotations;
  }

  private void rotationsToDistanceMeters(double leftPosAvg, double rightPosAvg, double leftPoseVelocity, double rightPosVelocity) {
    var leftPosition = Units.Meters.of(nativeUnitsToWheelRotations(leftPosAvg)).times(Math.PI).times(DriveTrainConstants.kWheelDiameter);
    var rightPosition = Units.Meters.of(nativeUnitsToWheelRotations(rightPosAvg)).times(Math.PI).times(DriveTrainConstants.kWheelDiameter);

    var leftVelocity = Units.MetersPerSecond.of(nativeUnitsToWheelRotations(leftPoseVelocity)).times(Math.PI).times(DriveTrainConstants.kWheelDiameter);
    var rightVelocity = Units.MetersPerSecond.of(nativeUnitsToWheelRotations(rightPosVelocity)).times(Math.PI).times(DriveTrainConstants.kWheelDiameter);

    m_leftPosition.mut_replace(leftPosition.magnitude(), Units.Meters);
    m_rightPosition.mut_replace(rightPosition.magnitude(), Units.Meters);
    m_leftVelocity.mut_replace(leftVelocity.magnitude(), Units.MetersPerSecond);
    m_rightVelocity.mut_replace(rightVelocity.magnitude(), Units.MetersPerSecond);
  }

  public void updateOdometry() {
    m_odometry.update(m_imu.getRotation2d(), m_rightPosition.magnitude(), m_leftPosition.magnitude());
  }

  public Command drive(CommandXboxController joystick) {
    return new RunCommand(() -> {
      final var xSpeed = DriveTrainConstants.kDriveSpeed.times(-joystick.getLeftY());
      final var rot = DriveTrainConstants.kTurnSpeed.times(-joystick.getRightX());
      drive(xSpeed, rot);
    }, this);
  }

  @Override
  public void periodic() {
    updateEncoders();
    updateOdometry();
  }
}