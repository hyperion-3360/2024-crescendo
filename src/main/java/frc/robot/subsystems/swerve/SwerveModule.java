package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private DutyCycleEncoder angleEncoder;
  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  private final VoltageOut m_voltageOutControl = new VoltageOut(0.0);
  private boolean m_debug = true;

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    /* Angle Encoder Config */
    angleEncoder = new DutyCycleEncoder(moduleConstants.magEncoderID);
    angleEncoder.setDutyCycleRange(
        RobotContainer.ctreConfigs.dutyCycleRangeMin, RobotContainer.ctreConfigs.dutyCycleRangeMax);
    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
    mAngleMotor.getConfigurator().apply(RobotContainer.ctreConfigs.swerveAngleFXConfig);
    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
    mDriveMotor.getConfigurator().apply(RobotContainer.ctreConfigs.swerveDriveFXConfig);
    mDriveMotor.getConfigurator().setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    if (m_debug)
      System.out.println(
          String.format(
              "Wheel: %d, speed: %f, angle : %f",
              moduleNumber, desiredState.speedMetersPerSecond, desiredState.angle.getDegrees()));
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      mDriveMotor.setControl(driveDutyCycle);
    } else {
      if (m_debug) System.out.println(String.format("setSpeed velocity : %s", driveVelocity));
      driveVelocity.Velocity =
          Conversions.MPSToRPS(
              desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
  }

  public void setDriveVoltage(double voltage) {
    m_voltageOutControl.withOutput(voltage);
    mDriveMotor.setControl(m_voltageOutControl);
  }

  public Rotation2d getMagEncoderPos() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    double magRotations = getMagEncoderPos().getRotations();
    if (m_debug)
      System.out.println(
          String.format(
              "Module : %d -> magRotations = %f, angle=%f",
              this.moduleNumber, magRotations, angleOffset.getRotations()));
    double absolutePosition = getMagEncoderPos().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
  }
}
