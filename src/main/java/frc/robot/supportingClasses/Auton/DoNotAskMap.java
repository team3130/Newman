package frc.robot.supportingClasses.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DoNothing;

import java.util.HashMap;
import java.util.Objects;

public class DoNotAskMap<T1, T2> extends HashMap<T1, T2> {
    public DoNotAskMap() {
        super();
    }

    public Object get(String key) {
        return Objects.requireNonNullElseGet(super.get(key), DoNothing::new);
    }
}
