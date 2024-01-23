// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WCPSwerveModule;

import static frc.robot.Constants.WCPSwerveModule.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.SwerveModule;

/* ######################################
  CHANGES FROM PHOENIX 5 TO PHOENIX 6:
  https://pro.docs.ctr-electronics.co/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#integral-zone-and-max-integral-accumulator
  ######################################
*/

public class WCPSwerveModule implements SwerveModule {

  // Subsystem parameters
  public static final double kNominalVolt = 10.0;
  public static final double kTickPerRotation = 28000.0;

  public static final double kPwmPeriod = 1.0 / 244.0;
  public static final double kPwmDutyMin = 1e-6 / kPwmPeriod;
  public static final double kPwmDutyMax = 4096e-6 / kPwmPeriod;

  private double m_encoderZero = 0.0;
  private boolean m_homed = false;
  private final double m_configZero;

  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final DutyCycleEncoder m_magEncoder;

  private final GenericEntry m_absAngleEntry;
  private final GenericEntry m_encOkEntry;
  private final GenericEntry m_motorEnc;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage m_angleRequest = new PositionVoltage(0);

  WCPSwerveModule(WCPSwerveModuleConfig config) {

    m_magEncoder = new DutyCycleEncoder(config.m_magEncoderChannel);
    m_magEncoder.setDistancePerRotation(kTickPerRotation);
    m_magEncoder.setDutyCycleRange(kPwmDutyMin, kPwmDutyMax);

    // drive motor config

    m_driveMotor = new TalonFX(config.m_driveMotorId);
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

    var Slot0Configs = new Slot0Configs();

    Slot0Configs.kP = kDriveKp;
    Slot0Configs.kI = kDriveKi;
    Slot0Configs.kD = kDriveKd;
    // Slot0Configs.kV = kDriveKv;

    m_driveMotor.getConfigurator().apply(Slot0Configs, 0.05);

    // m_driveMotor.config_IntegralZone(0, kDriveIZone); // Unnecessary in Phoenix 6
    // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#integral-zone-and-max-integral-accumulator

    final VoltageOut m_request = new VoltageOut(0);
    m_driveMotor.setControl(m_request.withOutput(kNominalVolt));
    m_velocityRequest.Slot = 0;

    // turn motor config

    m_turnMotor = new TalonFX(config.m_turnMotorId);
    m_turnMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_turnMotor.setInverted(true);

    m_turnMotor.setControl(m_request.withOutput(kNominalVolt));
    m_turnMotor.setNeutralMode(NeutralModeValue.Coast);

    var Slot1Configs = new Slot1Configs();

    Slot1Configs.kP = kTurnKp;
    Slot1Configs.kI = kTurnKi;
    Slot1Configs.kD = kTurnKd;

    m_turnMotor.getConfigurator().apply(Slot1Configs, 0.05);
    m_angleRequest.Slot = 1;
    // m_turnMotor.config_IntegralZone(0, kTurnIZone); // Unnecessary in Phoenix 6
    // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#integral-zone-and-max-integral-accumulator

    m_configZero = config.m_analogZero;

    final var absAngleEntryName = String.format("Abs Ch %d", config.m_magEncoderChannel);
    m_absAngleEntry =
        Shuffleboard.getTab("WCP Swerve Module").add(absAngleEntryName, 0.0).getEntry();

    final var encOkEntryName = String.format("Enc %d OK", config.m_magEncoderChannel);
    m_encOkEntry = Shuffleboard.getTab("Vitals").add(encOkEntryName, false).getEntry();

    final var motorEncoder = String.format("MotorEnc %d", config.m_turnMotorId);
    m_motorEnc = Shuffleboard.getTab("WCP Swerve Module").add(motorEncoder, 0.0).getEntry();
  }

  @Override
  public void periodic() {
    m_absAngleEntry.setDouble(m_magEncoder.getDistance());
    m_encOkEntry.setBoolean(m_magEncoder.isConnected());
    m_motorEnc.setDouble(m_turnMotor.getRotorPosition().getValueAsDouble());

    if (!m_homed) {
      // Home on first periodic loop so sensors are fully initialized

      // TODO: problem suspected! .getRotorPosition() is not in same units as magEncoder
      m_encoderZero =
          m_configZero
              + m_turnMotor.getRotorPosition().getValueAsDouble()
              - m_magEncoder.getDistance();
      m_homed = true;
    }
  }

  private Rotation2d getRotation() {

    double encoderDegrees = getEncoderDegrees();
    return Rotation2d.fromDegrees(encoderDegrees % 360);
  }

  @Override
  public SwerveModuleState getState() {

    return new SwerveModuleState(
        m_driveMotor.getVelocity().getValueAsDouble() * kTickToMeterPerS, this.getRotation());
  }

  @Override
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        m_driveMotor.getRotorPosition().getValueAsDouble() * kTickToMeter, this.getRotation());
  }

  private double getEncoderDegrees() {

    return (m_turnMotor.getRotorPosition().getValueAsDouble() - m_encoderZero) * kAnalogToDeg;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    var state = SwerveModuleState.optimize(desiredState, this.getRotation());
    var rotationDelta = state.angle.minus(this.getRotation());
    var setpointDegrees = getEncoderDegrees() + rotationDelta.getDegrees();

    // TODO: need to convert the state.speedMetersPerSecond * kMeterPerSToTick to RPS (revolution
    // per seconds)
    m_driveMotor.setControl(m_velocityRequest.withVelocity(0));

    // TODO: need to convert the setpointDegrees * kDegToAnalog + m_encoderZero to a new position
    // and velocity using a profile (see p. 46 of the above link)
    m_turnMotor.setControl(m_angleRequest);
  }
}
