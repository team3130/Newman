// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;


/**
 * A subsystem for manipulator.
 * Contains a Solenoid for the pneumatic pistons at the end of the mechanism.
 * Implements the Sendable object so that we can hurl data onto shuffleboard.
 */
public class HandGrabber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Solenoid grabber;
  /**
   * The default state of the solenoid (extended or not)
   */
  private final boolean defaultState = false;

  /**
   * Constructs a HandGrabber object that initializes the solenoid and sets it to its default value
   */
  public HandGrabber() {
    grabber = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM , Constants.PNM_Grabber);
    // default
    grabber.set(defaultState);

    // for the sendable logic
    SendableRegistry.add(this, "Manipulator");
  }

  /**
   * Toggles the grabber between true and false
   */
  public void toggleGrabber(){
    grabber.toggle();
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
    grabber.set(defaultState);
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
  public void extendGrabber() {
    grabber.set(defaultState);
  }

  /**
   * Retracts the grabber (Clamps which is the opposite of the default)
   */
  public void retractGrabber() {
    grabber.set(!defaultState);
  }

  /**
   * Get whether it is actuated or not.
   */
  public boolean getState() {
    return grabber.get() ^ defaultState;
  }

  /**
   * Set the solenoid to the passed in state
   * @param stateToSetTo actuate or to not actuate
   */
  public void setState(boolean stateToSetTo) {
    grabber.set(stateToSetTo ^ defaultState);
  }

  /**
   * Automatically gets called by shuffleboard
   * Calls the subsystemBase default sendable as well
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("state", this::getState, this::setState);
  }

}
