// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * A subsystem that contains a solenoid which actuates the pistons on the manipulator (also called grabber at times)
 */
public class Manipulator extends SubsystemBase {

  /**
   * the solenoid for the manipulator's pistons
   */
  private final Solenoid manipulator;

  /**
   * The default state of the solenoid (extended or not)
   */
  private final boolean defaultState = false;

  /**
   * Constructs a Manipulator object that initializes the solenoid and sets it to its default value
   */
  public Manipulator() {
    manipulator = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM , Constants.PNM_Grabber);
    // default
    manipulator.set(defaultState);

    // for the sendable logic
    // TODO this should be removed
    SendableRegistry.add(this, "Manipulator");
  }

  /**
   * Toggles the grabber between true and false
   */
  public void toggleManipulator(){
    manipulator.toggle();
  }

  /**
   * Periodic method that gets called repeatedly by the scheduler
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Stops the devices connected to this subsystem.
   * In this case it retracts the device
   */
  public void stop() {
    manipulator.set(defaultState);
  }

  /**
   * Periodic method that gets called by the schedule while in robot simulation
   */
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Extend the grabber (which would be open/default)
   */
  public void extend() {
    manipulator.set(defaultState);
  }

  /**
   * Retracts the grabber (Clamps which is the opposite of the default)
   */
  public void retract() {
    manipulator.set(!defaultState);
  }

  /**
   * Get whether it is actuated or not.
   */
  public boolean getState() {
    return manipulator.get() ^ defaultState;
  }

  /**
   * Set the solenoid to the passed in state
   * @param stateToSetTo actuate or to not actuate
   */
  public void setState(boolean stateToSetTo) {
    manipulator.set(stateToSetTo ^ defaultState);
  }

  /**
   * Automatically gets called by shuffleboard
   * Calls the subsystemBase default sendable as well
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Manipulator");
    builder.addBooleanProperty("grabber state", this::getState, this::setState);
  }
}
