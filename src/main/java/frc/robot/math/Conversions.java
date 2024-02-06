package frc.robot.math;

public class Conversions {
  // change the position of this file wherever you want I didn't add it to the constants just in
  // case
  /**
   * @param positionCounts CANCoder Position Counts
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double talonFXToDegrees(double positionCounts, double gearRatio) {
    return positionCounts * (360.0 / (gearRatio * 4096.0));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return CANCoder Position Counts
   */
  public static double degreesToTalonFX(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 4096.0));
  }

  /**
   * @param counts Falcon Position Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double positionCounts, double gearRatio) {
    return positionCounts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Position Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  /**
   * @param positionCounts Falcon Position Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Wheel
   * @return Meters
   */
  public static double falconToMeters(
      double positionCounts, double circumference, double gearRatio) {
    return positionCounts * (circumference / (gearRatio * 2048.0));
  }

  /**
   * @param meters Meters
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Wheel
   * @return Falcon Position Counts
   */
  public static double MetersToFalcon(double meters, double circumference, double gearRatio) {
    return meters / (circumference / (gearRatio * 2048.0));
  }

  // found this formula for swerve conversions and adapted it for NEO
  // https://www.chiefdelphi.com/t/converting-drive-motor-encoder-output-to-meters-swerve/428066
  /**
   * @param gearRatio the gear ration between the NEO 550 and the wheel
   * @param wheelDiameter the diameter of the wheel
   * @param encoderPosition the recorded position of the encoder
   * @return the wheel position in meters
   */
  public static double NEOToMeters(double gearRatio, double wheelDiameter, double encoderPosition) {
    double NEOpulsePerRotation = 42; // https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf

    double meterPerPulse = gearRatio * (Math.PI * wheelDiameter) / NEOpulsePerRotation;
    double wheelPositionInMeters = encoderPosition * meterPerPulse;
    return wheelPositionInMeters;
  }
}
