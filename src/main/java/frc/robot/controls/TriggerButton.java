package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TriggerButton extends JoystickButton {
    private final GenericHID stick;
    private final int axis;

    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public TriggerButton(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
        this.stick = joystick;
        this.axis = buttonNumber;
    }

    @Override
    public boolean getAsBoolean() {
        return stick.getRawAxis(axis) > 0.05;
    }

}
