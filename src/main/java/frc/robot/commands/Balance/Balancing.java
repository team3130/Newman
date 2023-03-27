package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;

public class Balancing extends SequentialCommandGroup {
    /**
     * Creates a new ComplexAuto.
     *
     * @param chassis The drive subsystem this command will run on
     */
    public Balancing(Chassis chassis) {
        Balance balance = new Balance(chassis);
        addCommands(
            //start on the driverstation
            balance,
            //Turn wheels sideways
            new RileyPark(chassis,balance.getDirection())
            );
    }
}
