// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * The intake subsystem which has a single solenoid which we actuate when we get to the human player station.
 * There is support for default states which uses XOR gates to determine what actuated and retracted is.
 */
public class Intake extends SubsystemBase {

  /**
   * The intake solenoid which fires both pistons on the intake
   */
  private final Solenoid m_solenoid;

  /**
   * This is the default state logic for the retracted and extended states of the pistons.
   * This gets XOr-ed with the desired state to yield retracted and extended values for the piston.
   */
  private final boolean defaultState;

  /**
   * Creates a new intake subsystem object.
   * Initializes a solenoid that controls air flow for the pistons.
   */
  public Intake() {
    m_solenoid = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM , Constants.PNM_Intake);

    // default should be whatever retracted is
    defaultState = false;
    m_solenoid.set(defaultState);
  }

  /**
   * Toggles the pnumatic for the pistons
   */
  public void toggle(){
    m_solenoid.toggle();
  }

  /**
   * Extends the pneumatic (opposite of retracted which we are saying is the default state)
   */
  public void extend() {
    m_solenoid.set(!defaultState);
  }

  /**
   * Retracts the small pneumatic (same as the default state)
   */
  public void retract() {
    m_solenoid.set(defaultState);
  }

  /**
   * @return the state of the small pneumatic (retracted should be false)
   */
  public boolean isExtended() {
    return m_solenoid.get() ^ defaultState;
  }

  /**
   * setter for the pneumatics state
   * @param state false is retracted, true is extended
   */
  private void setState(boolean state) {
    m_solenoid.set(state ^ defaultState);
  }

  /**
   * Initializes a sendable builder with the proper entry's on network tables.
   * Automatically gets called when **this** gets put on shuffleboard.
   * DOES NOT NEED TO BE CALLED BY USER.
   * @param builder sendable builder that gets entry's
   *                from {@link SubsystemBase#initSendable(SendableBuilder)} and the solenoid's state
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("solenoid", this::isExtended, this::setState);
  }
}
