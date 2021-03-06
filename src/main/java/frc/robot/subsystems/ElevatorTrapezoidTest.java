// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class ElevatorTrapezoidTest extends TrapezoidProfileSubsystem {
  private class ElevatorConstants {
    public final static double kGVolts = 0.38; // Volts
    public final static double kSVolts = 6-kGVolts; // Volts_that_take_to_move_the_elevator_from_rest - 
                                                            //               armkGVolts
    public final static double kVVoltSecondPerMeter = 39; // Volts * sec / radians
    public final static double kAVoltSecondSquaredPerMeter = 0.06; // Volts * sec^2 / radians
  }
  private TrapezoidProfile.State m_curPosVel = new TrapezoidProfile.State();
  private double m_curFFvalue = 0;
  private final  ElevatorFeedforward m_elevatorFF = new ElevatorFeedforward(
    ElevatorConstants.kSVolts, ElevatorConstants.kGVolts,
    ElevatorConstants.kVVoltSecondPerMeter, ElevatorConstants.kAVoltSecondSquaredPerMeter);
  /** Creates a new TrapezoidTests. */
  public ElevatorTrapezoidTest(double maxVel, double maxAcc, double startPos) {
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

  public ElevatorFeedforward getFFObj() {
    return m_elevatorFF;
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
    m_curFFvalue = m_elevatorFF.calculate(state.velocity);
  }
}
