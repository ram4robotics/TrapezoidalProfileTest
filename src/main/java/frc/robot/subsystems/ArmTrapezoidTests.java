// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class ArmTrapezoidTests extends TrapezoidProfileSubsystem {
  private class IntakeArmConstants {
    public final static double kGVolts = 1.61; // Volts
    public final static double kSVolts = 3.6-kGVolts; // armkSVolts = Volts_that_take_to_move_the_arm_from_rest - 
                                                            //               armkSVolts
    public final static double kVVoltSecondPerRad = 0.58; // Volts * sec / radians
    public final static double kAVoltSecondSquaredPerRad = 0.07; // Volts * sec^2 / radians
  }
  private TrapezoidProfile.State m_curPosVel = new TrapezoidProfile.State();
  private double m_curFFvalue = 0;
  private final ArmFeedforward m_armFF = new ArmFeedforward(
    IntakeArmConstants.kSVolts, IntakeArmConstants.kGVolts,
    IntakeArmConstants.kVVoltSecondPerRad, IntakeArmConstants.kAVoltSecondSquaredPerRad);
  /** Creates a new TrapezoidTests. */
  public ArmTrapezoidTests(double maxVel, double maxAcc, double startPos) {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(maxVel, maxAcc),
        // The initial position of the mechanism
        startPos);
  }

  public TrapezoidProfile.State getCurPosVel() {
    return m_curPosVel;
  }

  public double getCurFFvalue() {
    return m_curFFvalue;
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // If state did not change, do not apply power
    if (state.position == m_curPosVel.position) {
      m_curFFvalue = 0;
      return;
    }
    m_curPosVel = state;
    // Use the computed profile state here.
    m_curFFvalue = m_armFF.calculate(state.position, state.velocity);
  }
}
