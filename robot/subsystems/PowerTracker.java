// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerTracker extends SubsystemBase {
  private PowerDistribution m_PDP;
  private double voltage, temperatureCelsius, temperatureFarenheit, totalCurrent, totalPower, totalEnergy, numChannels, ID;
  /** Creates a new ExampleSubsystem. */
  public PowerTracker() {
    m_PDP = new PowerDistribution(5, ModuleType.kCTRE);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    voltage = m_PDP.getVoltage();
    temperatureCelsius = m_PDP.getTemperature();
    temperatureFarenheit = (temperatureCelsius*1.8)+32;
    totalCurrent = m_PDP.getTotalCurrent();
    totalPower = m_PDP.getTotalPower();
    totalEnergy = m_PDP.getTotalEnergy();
    numChannels = m_PDP.getNumChannels();
    ID = m_PDP.getModule();

    SmartDashboard.putNumber("Voltage", voltage);
    SmartDashboard.putNumber("Total Current", totalCurrent);
    SmartDashboard.putNumber("Total Power", totalPower);
    SmartDashboard.putNumber("Total Energy", totalEnergy);
    SmartDashboard.putNumber("Temperature (F)", temperatureFarenheit);
    SmartDashboard.putNumber("Number of Channels", numChannels);
    SmartDashboard.putNumber("PDP CAN ID", ID);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
