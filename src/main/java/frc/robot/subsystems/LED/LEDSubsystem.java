package frc.robot.subsystems.LED;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.sim.CANdleSimState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    public final CANdle candle = new CANdle(0, "CANivore");

    private final CANdleSimState candleSim = candle.getSimState();

    public LEDSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once every 20ms
        
    }
}