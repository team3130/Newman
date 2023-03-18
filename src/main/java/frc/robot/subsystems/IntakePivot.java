// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class IntakePivot extends SubsystemBase {
  private final Solenoid small;
  private final Solenoid large;

  private final boolean defaultStateSmall;
  private final boolean defaultStateLarge;

  public IntakePivot() {
    small = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM , Constants.PNM_SmallSolenoid);
    large = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM, Constants.PNM_LargeSolenoid);

    // default should be whatever retracted is (false?)
    defaultStateSmall = false;
    defaultStateLarge = false;

    small.set(defaultStateSmall);
    large.set(defaultStateLarge);
  }

  @Override
  public void periodic() {

  }

  public void goToNext(){
    if (!smallIsExtended() && !largeIsExtended()){ // at retracted which is in the bot
      // the next state is large retracted and small extended
      extendSmall();
    }
    else if (smallIsExtended() && !largeIsExtended()){ // at mid-position
      // the next state is large and small extended
      extendLarge();
    }
  }

  public void goToPrevious(){
    if (smallIsExtended() && largeIsExtended()){ // at extended which is outside of the bot
      // the next state is large retracted and small extended
      retractLarge();
    }
    else if (!largeIsExtended() && smallIsExtended()){
      retractSmall();
    }
  }

  /**
   * Toggles the small pneumatic
   */
  public void toggleSmall(){
    small.toggle();
  }

  /**
   * Toggles the large pneumatic
   */
  public void toggleLarge(){
    large.toggle();
  }

  /**
   * Extends the small pneumatic (opposite of retracted which we are saying is the default state)
   */
  public void extendSmall() {
    small.set(!defaultStateSmall);
  }

  /**
   * Extends the large pneumatic (opposite of retracted which we are saying is the default state)
   */
  public void extendLarge() {
    large.set(!defaultStateLarge);
  }

  /**
   * Retracts the large pneumatic (same as the default state)
   */
  public void retractLarge() {
    large.set(defaultStateLarge);
  }

  /**
   * Retracts the small pneumatic (same as the default state)
   */
  public void retractSmall() {
    small.set(defaultStateSmall);
  }

  /**
   * @return if we are at the middle position or not
   */
  public boolean atMiddlePos(){
    return smallIsExtended() && !largeIsExtended();
  }

  /**
   * @return the state of the large pneumatic (retracted should be false)
   */
  private boolean largeIsExtended() {
    return large.get() ^ defaultStateLarge;
  }

  /**
   * @return the state of the small pneumatic (retracted should be false)
   */
  private boolean smallIsExtended() {
    return small.get() ^ defaultStateSmall;
  }

  /**
   * setter for the large pneumatic
   * @param state false is retracted, true is extended
   */
  private void setLargeState(boolean state) {
    large.set(state ^ defaultStateLarge);
  }

  /**
   * setter for the small pneumatic
   * @param state false is retracted, true is extended
   */
  private void setSmallState(boolean state) {
    small.set(state ^ defaultStateSmall);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("large solenoid", this::largeIsExtended, this::setLargeState);
    builder.addBooleanProperty("small solenoid", this::smallIsExtended, this::setSmallState);
  }
}
