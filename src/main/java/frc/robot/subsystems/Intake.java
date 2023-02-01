//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private WPI_TalonSRX m_SpinnyBar;
    private Solenoid m_LargeSolenoid;
    private Solenoid m_SmallSolenoid;

    public Intake() {
        m_SpinnyBar = new WPI_TalonSRX(Constants.CAN_SpinnyBar);
        m_SpinnyBar.configFactoryDefault();
        m_SpinnyBar.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_LargeSolenoid = new Solenoid(Constants.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, Constants.PNM_LargeSolenoid);
        m_SmallSolenoid = new Solenoid(Constants.CAN_PNMMODULE, PneumaticsModuleType.CTREPCM, Constants.PNM_SmallSolenoid);
        this.m_SpinnyBar.setNeutralMode(NeutralMode.Coast);
    }

    public void setSpeed(double speed) {
        this.m_SpinnyBar.set(speed);

    }

    public void ToggleLPneumatic(){
        this.m_LargeSolenoid.toggle();
    }
    public void ToggleSPneumatic(){
        this.m_SmallSolenoid.toggle();
    }
}
