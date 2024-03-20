package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class SpeakerLock extends Command {
  private Swerve m_swerve;
  private Elevator m_elevator;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private boolean m_isLocked;
  private Translation3d m_target;
  private LEDs m_led;
  private Vision m_vision;
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  private Translation3d m_aprilTags[];
  private int m_alliance_index;
  private boolean m_isReady;
  private Double m_distanceFromSpeaker;
  private Double m_targetHeight;
  private double m_targetRot;
  private boolean m_elevatorReady;
  private int m_rotDirection = 1; // Anticlockwise = 1; clockwise = -1;

  private enum State{
    SEARCHING,
    CALCULATING,
    POSITIONNING
  };

  private State m_state = State.SEARCHING;
  
  /**
   * Command to keep the aiming at the speaker while keeping the robot in motion
   *
   * @param s_swerve swerve submodule instance
   * @param s_elevator elevator submodule instance
   * @param s_led led submodule instance
   * @param s_vision vision submodule instance
   * @param translationSup translation forward or backward
   * @param strafeSup moving laterally
   */
  public SpeakerLock(
      Swerve s_swerve,
      Elevator s_elevator,
      LEDs s_led,
      Vision s_vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup) {

    this.m_swerve = s_swerve;
    this.m_elevator = s_elevator;
    this.m_led = s_led;
    this.m_vision = s_vision;

    // load the list of april tags from the field
    try {
      m_aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      m_aprilTagFieldLayout = null;
    }

    if (m_aprilTagFieldLayout != null) {

      // only allocate enough space for both speakers (1 per alliance side)
      m_aprilTags = new Translation3d[Constants.VisionConstants.kSpeakerIndex.length];

      // convert translation3d to translation2d for speakers position
      int i = 0;
      for (var tagId : Constants.VisionConstants.kSpeakerIndex) {
        var tagPose = m_aprilTagFieldLayout.getTagPose(tagId).get();
        // no need for Z
        m_aprilTags[i] = new Translation3d(tagPose.getX(), tagPose.getY(), tagPose.getZ());
        i++;
      }

      // find out the current alliance with fail safe
      m_alliance_index = 0;
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) m_alliance_index = alliance.get() == Alliance.Red ? 0 : 1;
    }

    addRequirements(s_swerve);
    addRequirements(s_elevator);
    addRequirements(s_led);
    addRequirements(s_vision);

    this.m_translationSup = translationSup;
    this.m_strafeSup = strafeSup;
    this.m_isLocked = false;
  }

  @Override
  public void initialize() {
    // good idea to unlock when the command is first scheduled! :)
    m_isLocked = false;
    m_state = State.SEARCHING;
    m_isReady = false;
    m_distanceFromSpeaker = null;
    m_targetHeight = null;
    m_targetRot = 0;
    m_elevatorReady = false;
    m_rotDirection = 1;
  }

  @Override
  public boolean isFinished(){
    return m_isReady;
  }

  @Override
  public void end(boolean interrupted){    
    m_swerve.drive(
                new Translation2d(0,0),
                0,
                false,
                true);
    m_elevator.extendTheElevator(Elevator.elevatorHeight.LOW);
    // m_led.setState(LEDs.State.IDLE);
  }

  @Override
  public void execute() {
    double translationVal =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);

    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = m_swerve.getRotation2d().getRadians();

    if (m_aprilTagFieldLayout != null) {
      SmartDashboard.putBoolean("Locked on speaker", m_isLocked);
      if (m_isLocked) {
        if(m_vision.isValidPos() &&  m_state == State.CALCULATING){// TODO: recalculate everytime when an april tag is available 
          final var cameraPose2d = m_vision.getPosition();
          var camTranslation = new Translation2d(cameraPose2d.getX(), cameraPose2d.getY());
        
          // compute rotation to apply to face the target
          Translation2d pointToFace = new Translation2d(m_target.getX(), m_target.getY()).times(100);
        
          m_targetHeight = m_target.getZ();// - Constants.VisionConstants.kCameraHeight;

          // distance from camera to tag -- get distance from robot
          final var dx = camTranslation.getX() - pointToFace.getX();
          final var dy = camTranslation.getY() - pointToFace.getY();
          
          final var hyp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
          m_distanceFromSpeaker = Math.cos(Constants.VisionConstants.kShooterCameraPitch) * hyp / 100; // in meters
          final var wantedRot = Math.atan(Math.abs(dy)/Math.abs(dx));

          // Calulate the rotation needed to face the tag
          // Correct for camera position. Anticlockwise rotation
          double camRot = cameraPose2d.getRotation().getRadians();
          
          rotationVal = camRot;// + Constants.VisionConstants.kShooterCameraPitch;
          if(rotationVal > 0){
            m_rotDirection = -1;
          }
          

          rotationVal =
              Math.abs(Math.abs(rotationVal) - Math.abs(wantedRot)) * m_rotDirection;

          // Convert to absolute rotation based on the odometry
          m_targetRot = m_swerve.getRotation2d().getRadians() + rotationVal;


          m_state = State.POSITIONNING;
        }
        if(m_state == State.POSITIONNING){

          var currentRot = m_swerve.getRotation2d().getRadians();

          //TODO: stop if over rotated too. NEED TO CHECK DIRECTION OF MOVEMENT
          if (Math.abs(Math.abs(currentRot)-Math.abs(m_targetRot)) < 0.05) { // ~3deg margin
            m_isReady = true;
          }else{
            m_isReady = false;
          }
          
          // compute elevator angle to shoot in target, assuming straightline not parabol
          if (m_distanceFromSpeaker > Constants.ShooterConstants.kMaxShootingDistance)
            m_led.setState(LEDs.State.PREPARE_SHOT_SPEAKER);
          else if(!m_elevatorReady){
            // double elevatorAngle = Math.atan2(m_targetHeight, m_distanceFromSpeaker);
            // SmartDashboard.putNumber("Elevator Angle", elevatorAngle);
            // m_elevator.extendTheElevator(elevatorAngle);
            
            double elevatorPosition = 332.169-146.57*Math.pow(m_distanceFromSpeaker*100, 0.124133);
            m_elevator.rawPosition(elevatorPosition);
            m_led.setState(LEDs.State.SHOOT_READY_SPEAKER);
            m_elevatorReady = true;
          }
        }
      
      } else { // don't have a lock yet so look at possible target from vision
        int selecteg_tag = -1;
        if (m_vision.isValidPos()) {
          for (var t : m_vision.getVisibleTagIds())
            if (t == Constants.VisionConstants.kSpeakerIndex[m_alliance_index]) {
              selecteg_tag = Constants.VisionConstants.kSpeakerIndex[m_alliance_index];
              break;
            }

          // still possible that the visible ids are not the ones we're interested in
          if (selecteg_tag != -1) {
            // TODO: optimize this.. Meh recreating object each time...¯\_(ツ)_/¯
            final var tagIdx =
                Arrays.asList(Constants.VisionConstants.kSpeakerIndex).indexOf(selecteg_tag);
            m_target = m_aprilTags[tagIdx];
            m_isLocked = true;
            m_state = State.CALCULATING;
          } else m_led.setState(LEDs.State.PREPARE_SHOT_SPEAKER);
        }
      }

      /* Rotate to face speaker */
      m_swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                (!m_isReady && m_state == State.POSITIONNING ? m_rotDirection*0.2 : 0)  * Constants.Swerve.maxAngularVelocity, // constant speed
                false,
                true);
    }
  }
}
