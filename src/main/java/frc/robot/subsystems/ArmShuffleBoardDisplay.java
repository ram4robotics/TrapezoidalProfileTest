// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmShuffleBoardDisplay extends SubsystemBase {
  public ArmTrapezoidTests m_trapTests;
  boolean m_trapTestsCreated = false;
  private double m_maxVel, m_maxAcc, m_startPos, m_endPos;
  /** Creates a new ShuffleBoardDisplay. */
  public ArmShuffleBoardDisplay() {
    m_maxVel = 1.0;
    m_maxAcc = 1.0;
    m_startPos = 0;
    m_endPos = 130;
    SmartDashboard.putNumber("Max Velocity", m_maxVel);
    SmartDashboard.putNumber("Max Acceleration", m_maxAcc);
    SmartDashboard.putNumber("Starting Position", m_startPos);
    SmartDashboard.putNumber("End Position", m_endPos);
  }

  public void createNewTrapezoidProfile() {
    m_maxVel = SmartDashboard.getNumber("Max Velocity", m_maxVel);
    m_maxAcc = SmartDashboard.getNumber("Max Acceleration", m_maxAcc);
    m_startPos = SmartDashboard.getNumber("Starting Position", m_startPos);
    m_endPos = SmartDashboard.getNumber("End Position", m_endPos);
    m_trapTests = new ArmTrapezoidTests(m_maxVel, m_maxAcc, Units.degreesToRadians(m_startPos));
    m_trapTestsCreated = true;
    m_trapTests.setGoal(new TrapezoidProfile.State(Units.degreesToRadians(m_endPos), 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_trapTestsCreated)
      return;
    SmartDashboard.putNumber("Current Position", m_trapTests.getCurPosVel().position);
    SmartDashboard.putNumber("Current Velocity", m_trapTests.getCurPosVel().velocity);
    SmartDashboard.putNumber("Current Feed Forward", m_trapTests.getCurFFvalue());
  }
}
