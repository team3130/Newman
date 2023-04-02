// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class Intake extends SubsystemBase {
  private final Solenoid intake;

  private final boolean defaultStateSmall;

  public Intake() {
    intake = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM , Constants.PNM_Intake);

    // default should be whatever retracted is
    defaultStateSmall = false;

    intake.set(defaultStateSmall);
  }

  @Override
  public void periodic() {

  }

  /**
   * Toggles the small pneumatic
   */
  public void toggleSmall(){
    intake.toggle();
  }

  /**
   * Extends the small pneumatic (opposite of retracted which we are saying is the default state)
   */
  public void extendSmall() {
    intake.set(!defaultStateSmall);
  }


  /**
   * Retracts the small pneumatic (same as the default state)
   */
  public void retractSmall() {
    intake.set(defaultStateSmall);
  }

  /**
   * @return the state of the small pneumatic (retracted should be false)
   */
  public boolean isExtended() {
    return intake.get() ^ defaultStateSmall;
  }

  /**
   * setter for the small pneumatic
   * @param state false is retracted, true is extended
   */
  private void setSmallState(boolean state) {
    intake.set(state ^ defaultStateSmall);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("small solenoid", this::isExtended, this::setSmallState);
  }
}
