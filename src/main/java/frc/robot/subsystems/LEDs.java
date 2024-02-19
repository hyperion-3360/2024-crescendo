// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  /** each LED color has its own PWM output */
  private PWM m_redLED, m_greenLED, m_blueLED, m_whiteLED;

  private PowerDistribution m_reverb = new PowerDistribution(20, ModuleType.kRev);

  private boolean m_debug = false;

  /** ledValue structure to easily manipulate LED values in 8bit RGB */
  class ledValue {
    public int r, g, b, w;
    static final int maxValue = 4095;
    static final int max8bit = 255;

    ledValue(int r, int g, int b, int w) {
      this.r = r;
      this.g = g;
      this.b = b;
      this.w = w;
    }

    static int convertToPulse(int val8bit) {
      return ((val8bit % (max8bit + 1)) * maxValue) / max8bit;
    }

    @Override
    public String toString() {
      return String.format("R: %d, G: %d, B: %d, W: %d", r, g, b, w);
    }
  }
  ;

  public enum State {
    IDLE, // Idle orange, default
    INTAKE_ROLLING, // Intake white slow flash, when intake is rolling
    NOTE_INSIDE, // Detected note green, note in beam cutter triggered
    PREPARE_SHOT_SPEAKER, // Aim activated white quick flash, with vision aim function running
    SHOOT_READY_SPEAKER, // Aim ready blue, vision aim lock
    SHOT_DONE, // Shot done green, when shooting sequence is completed note has been shot
    CLIMBING, // Climb yellow, during the entire climb sequence
    PREPARE_SHOT_AMP, // Aim activated pale yellow quick flash, with vision aim function running
    SHOOT_READY_AMP, // Aim ready deep purple, vision aim lock
    TRAP_HAS_NOTE, // Limit switch activated, purple
  }

  /* current LED state */
  private State m_ledState = State.IDLE;

  /* used colors defined in RGB */
  private ledValue kOrange = new ledValue(255, 165, 0, 0);
  private ledValue kWhite = new ledValue(0, 0, 0, 255);
  private ledValue kBlue = new ledValue(0, 0, 255, 0);
  private ledValue kGreen = new ledValue(0, 255, 0, 0);
  private ledValue kpurple = new ledValue(255, 0, 255, 0);
  private ledValue kDark = new ledValue(0, 0, 0, 0);

  /* current ledvalue, used for blinking */
  private ledValue m_currentValue;

  private int kSlowFlashingDelay = (1000000 / 4); // 4 times per second delay in usec
  private int kFastFlashingDelay = (1000000 / 10); // 10 times per second delay in usec

  /* instance member for singleton implementation */
  private static LEDs m_instance = null;

  /* next toggle time for blinking */
  long m_flashDuration = 0;

  /** Static function that creates and LED instance if it doesn't exists yet */
  public static LEDs getInstance() {
    /**** TODO: NOT THREADS SAFE ****/
    if (m_instance == null) {
      m_instance =
          new LEDs(
              Constants.LED.kLEDredPWMPort,
              Constants.LED.kLEDgreenPort,
              Constants.LED.kLEDbluePort,
              Constants.LED.kLEDwhitePort);
    }
    return m_instance;
  }

  private LEDs(int redPort, int greenPort, int bluePort, int whitePort) {
    m_redLED = new PWM(redPort);
    m_greenLED = new PWM(greenPort);
    m_blueLED = new PWM(bluePort);
    m_whiteLED = new PWM(whitePort);

    // TODO: is this needed?
    // redLED.enableDeadbandElimination(true);
    // greenLED.enableDeadbandElimination(true);
    // blueLED.enableDeadbandElimination(true);
    // whiteLED.enableDeadbandElimination(true);
    setState(State.IDLE);
  }

  public void setState(State state) {
    // if (m_debug) System.out.println(String.format("New sate: %s", state));
    m_ledState = state;

    switch (m_ledState) {
      case IDLE: // orange
        m_flashDuration = 0;
        m_currentValue = kOrange;
        break;
      case INTAKE_ROLLING: // flashing white
        m_currentValue = kWhite;
        m_flashDuration = RobotController.getFPGATime() + kSlowFlashingDelay;
        break;
      case NOTE_INSIDE:
        m_flashDuration = 0;
        m_currentValue = kGreen;
        break;
      case PREPARE_SHOT_SPEAKER: // flashing white
        m_currentValue = kWhite;
        m_flashDuration = RobotController.getFPGATime() + kFastFlashingDelay;
        break;
      case SHOOT_READY_SPEAKER:
        m_flashDuration = 0;
        m_currentValue = kBlue;
        break;
      case SHOT_DONE:
        m_flashDuration = 0;
        m_currentValue = kGreen;
        break;
      case CLIMBING:
        m_flashDuration = 0;
        m_currentValue = kpurple;
        break;
      case TRAP_HAS_NOTE:
        m_flashDuration = 0;
        m_currentValue = kpurple;
        break;
      default:
        break;
    }
    setRGB(m_currentValue);
  }

  /* the periodic function is used for the blinking behavior */
  @Override
  public void periodic() {
    long currentTime = RobotController.getFPGATime();
    switch (m_ledState) {
      case INTAKE_ROLLING:
        if (currentTime > m_flashDuration) {
          m_currentValue = m_currentValue == kWhite ? kDark : kWhite;
          m_flashDuration = RobotController.getFPGATime() + kSlowFlashingDelay;
          setRGB(m_currentValue);
        }
        break;
      case PREPARE_SHOT_SPEAKER:
        if (currentTime > m_flashDuration) {
          m_currentValue = m_currentValue == kWhite ? kDark : kWhite;
          m_flashDuration = RobotController.getFPGATime() + kFastFlashingDelay;
          setRGB(m_currentValue);
        }
        break;
      case TRAP_HAS_NOTE:
        if (currentTime > m_flashDuration) {
          m_currentValue = m_currentValue == kpurple ? kDark : kpurple;
          m_flashDuration = RobotController.getFPGATime() + kSlowFlashingDelay;
          setRGB(m_currentValue);
        }
        break;
      default:
        break;
    }

    m_reverb.setSwitchableChannel(DriverStation.isEnabled());
  }

  /**
   * directly control each LED PWM output based on the 8bit values and convert them in the range
   * 0-4096
   *
   * @param val 8bit LED value
   */
  private void setRGB(ledValue val) {
    // if (m_debug) System.out.println(val);
    m_redLED.setPulseTimeMicroseconds(ledValue.convertToPulse(val.r));
    m_greenLED.setPulseTimeMicroseconds(ledValue.convertToPulse(val.g));
    m_blueLED.setPulseTimeMicroseconds(ledValue.convertToPulse(val.b));
    m_whiteLED.setPulseTimeMicroseconds(ledValue.convertToPulse(val.w));
  }

  public Command test() {
    return this.runOnce(() -> setState(State.IDLE))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.NOTE_INSIDE))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.INTAKE_ROLLING))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.SHOOT_READY_SPEAKER))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.SHOT_DONE))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.PREPARE_SHOT_SPEAKER))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.CLIMBING))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.PREPARE_SHOT_AMP))
        .andThen(new WaitCommand(3))
        .andThen(() -> setState(State.SHOOT_READY_AMP))
        .andThen(new WaitCommand(3));
  }

  public Command reverbOff() {
    return this.runOnce(() -> m_reverb.setSwitchableChannel(false));
  }
}
