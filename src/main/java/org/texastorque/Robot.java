package org.texastorque;

import java.util.ArrayList;

import org.texastorque.inputs.State;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.subsystems.DriveBase;
import org.texastorque.subsystems.Subsystem;
import org.texastorque.torquelib.base.TorqueIterative;


public class Robot extends TorqueIterative {

  // Subsystems
  private ArrayList<Subsystem> subsystems = new ArrayList<>();
  private DriveBase driveBase = DriveBase.getInstance();

  // Input
  private State state = State.getInstance();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    loadSubsystems();
  }

  /**
   * Add the subsytems to the ArrayList
   */
  private void loadSubsystems() {
    subsystems.add(driveBase);
  }

  /**
   * Set the robot state to teleop and run initalization code
   */
  @Override
  public void teleopInit() {
    state.setRobotState(RobotState.TELEOP);
    subsystems.forEach(s->s.initTeleop());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Always update SmartDashboard
   */
  @Override
  public void alwaysContinuous() {
    subsystems.forEach(c->c.smartDashboard());  
  }

  @Override
  public void endCompetition() {

  }
}
