// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.WristIntake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ReverseWristAutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristIntake m_WristIntake;
  private boolean reversed = false;
  public boolean intakeIn;
  private javax.management.timer.Timer intakeTimer = new Timer();
  private double time = intakeTimer.get();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReverseWristAutoCommand(WristIntake wristintake, boolean intakeIn) {
    m_WristIntake = wristintake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeIn == true && time < 1) {
      m_WristIntake.wrist(true, false);
    }
    else if(intakeIn == false && time < 1) {
      m_WristIntake.wrist(false, true);
    }
    reversed = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(reversed == true) {
      return true;
    }
    else {
      return false;
    }
  }
}