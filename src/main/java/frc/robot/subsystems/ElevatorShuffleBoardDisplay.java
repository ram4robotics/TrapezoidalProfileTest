// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorShuffleBoardDisplay extends SubsystemBase {
  public ElevatorTrapezoidTest m_trapTests;
  boolean m_trapTestsCreated = false;
  private double m_maxVel, m_maxAcc, m_startPos, m_endPos;
  /** Creates a new ShuffleBoardDisplay. */
  public ElevatorShuffleBoardDisplay() {
    m_maxVel = 0.152; // meters / sec
    m_maxAcc = 1.2; // meters / sec^2
    m_startPos = 0; // inches
    m_endPos = 20; // inches
    SmartDashboard.putNumber("Max Velocity", m_maxVel);
    SmartDashboard.putNumber("Max Acceleration", m_maxAcc);
    SmartDashboard.putNumber("Starting Height", m_startPos);
    SmartDashboard.putNumber("End Height", m_endPos);
  }

  public void createNewTrapezoidProfile() {
    m_maxVel = SmartDashboard.getNumber("Max Velocity", m_maxVel);
    m_maxAcc = SmartDashboard.getNumber("Max Acceleration", m_maxAcc);
    m_startPos = SmartDashboard.getNumber("Starting Height", m_startPos);
    m_endPos = SmartDashboard.getNumber("End Height", m_endPos);
    m_trapTests = new ElevatorTrapezoidTest(m_maxVel, m_maxAcc, Units.inchesToMeters(m_startPos));
    m_trapTestsCreated = true;
    m_trapTests.setGoal(new TrapezoidProfile.State(Units.inchesToMeters(m_endPos), 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_trapTestsCreated)
      return;
    SmartDashboard.putNumber("Current Height", m_trapTests.getCurPosVel().position);
    SmartDashboard.putNumber("Current Velocity", m_trapTests.getCurPosVel().velocity);
    SmartDashboard.putNumber("Current Feed Forward", m_trapTests.getCurFFvalue());
    SmartDashboard.putNumber("Max Achievable Velovity", m_trapTests.getFFObj().maxAchievableVelocity(12, m_maxAcc));
    SmartDashboard.putNumber("Min Achievable  Velocity", m_trapTests.getFFObj().minAchievableVelocity(12, m_maxAcc));
    SmartDashboard.putNumber("Max Achievable Acceleration", m_trapTests.getFFObj().maxAchievableAcceleration(12, m_maxVel));
    SmartDashboard.putNumber("Min Achievable Acceleration", m_trapTests.getFFObj().minAchievableAcceleration(12, m_maxVel));
  }
}
