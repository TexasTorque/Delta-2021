package org.texastorque;

import java.util.ArrayList;

import org.texastorque.auto.AutoManager;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.subsystems.Climber;
import org.texastorque.subsystems.DriveBase;
import org.texastorque.subsystems.Intake;
import org.texastorque.subsystems.Magazine;
import org.texastorque.subsystems.Shooter;
import org.texastorque.subsystems.Subsystem;
import org.texastorque.torquelib.base.TorqueIterative;

public class Robot extends TorqueIterative {

  // Subsystems
  private ArrayList<Subsystem> subsystems = new ArrayList<>();
  private DriveBase driveBase = DriveBase.getInstance();
  private Intake intake = Intake.getInstance();
  private Magazine magazine = Magazine.getInstance();
  private Climber climber = Climber.getInstance();
  private Shooter shooter = Shooter.getInstance();

  // Input
  private Feedback feedback = Feedback.getInstance();
  private Input input = Input.getInstance();
  private State state = State.getInstance();

  // Misc
  private AutoManager autoManager = AutoManager.getInstance();

  /**
   * Load the subsytems when the robot first starts
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
    subsystems.add(intake);
    subsystems.add(magazine);
    subsystems.add(climber);
    subsystems.add(shooter);
    autoManager.displayChoices();
  }

  /**
   * Set the robot state to teleop and run initalization code
   */
  @Override
  public void teleopInit() {
    state.setRobotState(RobotState.TELEOP);
    subsystems.forEach(s -> s.initTeleop());
  }

  /**
   * Continuously update input and run teleop commands
   */
  @Override
  public void teleopContinuous() {
    switch (state.getRobotState()) {
      default:
        input.update();
    }

    subsystems.forEach(s -> s.runTeleop(state.getRobotState()));
  }

  /**
   * Initialize auto
   */
  @Override
  public void autoInit() {
    state.setRobotState(RobotState.AUTO);
    autoManager.chooseCurrentSequence();
    subsystems.forEach(s -> s.initAuto());
  }

  /**
   * Run auto mode
   */
  @Override
  public void autoContinuous() {
    autoManager.runCurrentSequence();
    subsystems.forEach(c -> c.runAuto(RobotState.AUTO));
  }

  /**
   * Run disable code for each subsystem
   */
  @Override
  public void disabledInit() {
    subsystems.forEach(s -> s.disable());
  }

  /**
   * Always update SmartDashboard
   */
  @Override
  public void alwaysContinuous() {
    feedback.update();
    autoManager.displayChoices();
    subsystems.forEach(c -> c.smartDashboard());
  }

  @Override
  public void endCompetition() {
  }
}
