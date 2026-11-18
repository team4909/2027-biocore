package frc.robot.subsystems.LED;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
//IMPORTANT!!! candle.setControl(new SolidColor(startIndex, endIndex).withColor(new RGBWColor(r, g, b, 0)));
public class CANdleSubsystem extends SubsystemBase {
    private CANdle candle;
    private final int totalLEDs = 110;//+ 8 for the candle //SRIHAAS!!! CHANGE THIS TO THE NUMBER OF LEDS YOU HAVE! (Made it a multiple of 4)
    private final int frontStrip = 45;
    private final int leftStrip = 19;
    private final int backStrip = 19;
    private final int rightStrip = 19;
    private final int candleLEDs = 110;



    public CANdleSubsystem(int canID, DoubleSupplier speedSupplier) {
        candle = new CANdle(0);

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.7; 
        candle.getConfigurator().apply(config);
    }

    // public void setTeleopZone(int ZoneIndex, int r, int g, int b) {   
    // }

    // public void setAutoZone(int AutoZoneIndex, int r, int g, int b) {
    // }

    // public void setDisabledZone(int DisabledZoneIndex, int r, int g, int b) {
    // }

    public void setSolidColor(int r, int g, int b) {
        candle.setControl(
            new SolidColor(8, 8 + totalLEDs - 1)                
            .withColor(new RGBWColor(r, g, b, 0))
        );
    }

    public void clearLEDs() {
        setSolidColor(0, 0, 0);
    }
    
    @Override
    public void periodic() {
            double batteryVoltage = Math.round(RobotController.getBatteryVoltage() * 10.0) / 10.0;
    
            private int frontLedsOffPer = (int) Math.ceil(frontStrip / 40.0);
            private int leftLedsOffPer = (int) Math.ceil(leftStrip / 40.0);
            private int backLedsOffPer = (int) Math.ceil(backStrip / 40.0);
            private int rightLedsOffPer = (int) Math.ceil(rightStrip / 40.0); 

            private int frontEndIndex = (int) (batteryVoltage * frontLedsOffPer);
            private int leftEndIndex = (int) (batteryVoltage * leftLedsOffPer);
            private int backEndIndex = (int) (batteryVoltage * backLedsOffPer);
            private int rightEndIndex = (int) (batteryVoltage * rightLedsOffPer);
        if (DriverStation.isDisabled()) {
            runDisabledLEDs();
        } else if (DriverStation.isAutonomous()) {
            runAutoLEDs();
        } else {
            runTeleopLEDs();
        }
    }

    //Disabled functions
    private void runDisabledLEDs() {

        candle.setControl(
            new SolidColor(8,44)                
            .withColor(new RGBWColor(0, 0, 255, 0))
        );

        candle.setControl(
            new SolidColor(53,44)                
            .withColor(new RGBWColor(0, 0, 255, 0))
        );

        candle.setControl(
            new SolidColor(72,44)                
            .withColor(new RGBWColor(0, 0, 255, 0))
        );

        candle.setControl(
            new SolidColor(91,44)                
            .withColor(new RGBWColor(0, 0, 255, 0))
        );

    }

    //Autonomous functions
    private void runAutoLEDs() {        
    }

    //Teleop functions
    private void runTeleopLEDs() {
    }

}