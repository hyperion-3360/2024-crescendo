package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;

public class AimLock extends Command {
  private Swerve m_swerve;
  private Elevator m_elevator;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private boolean m_isLocked;
  private Translation2d m_target;
  private double m_targetHeight;
  private LEDs m_led;

  /**
   * Command to keep the aim of a target while keeping the robot in motion
   *
   * @param s_swerve swerve submodule instance
   * @param translationSup translation forward or backward
   * @param strafeSup moving laterally
   */
  public AimLock(
      Swerve s_swerve,
      Elevator s_elevator,
      LEDs s_led,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup) {

    this.m_swerve = s_swerve;
    this.m_elevator = s_elevator;
    this.m_led = s_led;

    addRequirements(s_swerve);
    addRequirements(s_elevator);
    addRequirements(s_led);

    this.m_translationSup = translationSup;
    this.m_strafeSup = strafeSup;
    this.m_isLocked = false;
  }

  @Override
  public void execute() {
    double translationVal =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);

    double rotationVal = m_swerve.getRotation2d().getRadians();

    if (m_isLocked) {
      // compute rotation to apply to face the target
      Translation2d pointToFace = m_target;
      Rotation2d rotationNeeded = pointToFace.minus(m_swerve.getPose().getTranslation()).getAngle();
      rotationVal = rotationNeeded.getRadians();

      // compute elevator angle to shoot in target
      double distance2Target = m_swerve.getPose().getTranslation().getDistance(pointToFace);
      if (distance2Target > Constants.ShooterConstants.kMaxShootingDistance)
        m_led.setState(LEDs.State.PREPARE_SHOT_SPEAKER);
      else {
        double elevatorAngle = Math.atan2(m_targetHeight, distance2Target);
        m_elevator.extendTheElevator(elevatorAngle);
        m_led.setState(LEDs.State.SHOOT_READY_SPEAKER);
      }
    }
    // look at possible target from vision
    else {

    }

    /* Drive */
    m_swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        false,
        true);
  }
}
