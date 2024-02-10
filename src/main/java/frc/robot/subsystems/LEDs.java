// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private static PWM m_redLED;
  private static PWM m_greenLED;
  private static PWM m_blueLED;
  private static PWM m_whiteLED;
  private double m_lastToggleTime = 0;
  private boolean m_whiteToggle = false;

  class ledValue {
    public int r, g, b, w;

    ledValue(int r, int g, int b, int w) {
      this.r = r;
      this.g = g;
      this.b = b;
      this.w = w;
    }
  }
  ;

  /*
   * LED STATES DOCUMENTATION
   * Intake white slow flash, when intake is rolling
   * Detected note green, note in beam cutter triggered
   * Aim activated white quick flash, with vision aim function running
   * Aim ready blue, vision aim lock
   * Shot done green, when shooting sequence is completed note has been shot
   * Climb yellow, during the entire climb sequence
   */

  public enum State {
    IDLE, // Idle orange, default
    INTAKE_ROLLING, // Intake white slow flash, when intake is rolling
    NOTE_INSIDE, // Detected note green, note in beam cutter triggered
    PREPARE_SHOT, // Aim activated white quick flash, with vision aim function running
    SHOOT_READY, // Aim ready blue, vision aim lock
    SHOT_DONE, // Shot done green, when shooting sequence is completed note has been shot
    CLIMBING, // Climb yellow, during the entire climb sequence
  }

  private State m_ledState = State.IDLE;

  private ledValue kOrange = new ledValue(255, 165, 0, 0);
  private ledValue kWhite = new ledValue(0, 0, 0, 255);
  private ledValue kBlue = new ledValue(0, 0, 255, 0);
  private ledValue kGreen = new ledValue(0, 255, 0, 0);
  private ledValue kYellow = new ledValue(255, 255, 0, 0);
  private ledValue kDark = new ledValue(0, 0, 0, 0);

  private ledValue m_currentValue;

  private int kSlowFlashingDelay = (1000000 / 4); // 4 times per second delay in usec
  private int kFastFlashingDelay = (1000000 / 8); // 8 times per second delay in usec

  private static LEDs m_instance = null;

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

    // redLED.enableDeadbandElimination(true);
    // greenLED.enableDeadbandElimination(true);
    // blueLED.enableDeadbandElimination(true);
    // whiteLED.enableDeadbandElimination(true);
    setState(State.IDLE);
  }

  public void setState(State state) {
    m_ledState = state;

    // Set the new LED pattern
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
      case PREPARE_SHOT: // flashing white
        m_currentValue = kWhite;
        m_flashDuration = RobotController.getFPGATime() + kFastFlashingDelay;
        break;
      case SHOOT_READY:
        m_flashDuration = 0;
        m_currentValue = kBlue;
        break;
      case SHOT_DONE:
        m_flashDuration = 0;
        m_currentValue = kGreen;
        break;
      case CLIMBING:
        m_flashDuration = 0;
        m_currentValue = kYellow;
        break;
      default:
        break;
    }
    setRGB(m_currentValue);
  }

  @Override
  public void periodic() {
    switch (m_ledState) {
      case INTAKE_ROLLING:
        if (RobotController.getFPGATime() > m_flashDuration) {
          m_currentValue = m_currentValue == kWhite ? kDark : kWhite;
          m_flashDuration = RobotController.getFPGATime() + kSlowFlashingDelay;
          setRGB(m_currentValue);
        }
        break;
      case PREPARE_SHOT:
        if (RobotController.getFPGATime() > m_flashDuration) {
          m_currentValue = m_currentValue == kWhite ? kDark : kWhite;
          m_flashDuration = RobotController.getFPGATime() + kFastFlashingDelay;
          setRGB(m_currentValue);
        }
        break;
      default:
        break;
    }
  }

  private void setRGB(ledValue val) {
    m_redLED.setPulseTimeMicroseconds((val.r * 4096) / 255);
    m_greenLED.setPulseTimeMicroseconds((val.g * 4096) / 255);
    m_blueLED.setPulseTimeMicroseconds((val.b * 4096) / 255);
    m_whiteLED.setPulseTimeMicroseconds((val.w * 4096) / 255);
  }
}
