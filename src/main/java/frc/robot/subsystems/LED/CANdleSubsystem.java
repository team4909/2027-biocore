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
    private final double max = 12.8;
    private final double min = 10;

    private int frontEndIndex;
    private int leftEndIndex;
    private int backEndIndex;
    private int rightEndIndex;



    public CANdleSubsystem(int canID, DoubleSupplier speedSupplier) {
        candle = new CANdle(canID);

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.7; 
        candle.getConfigurator().apply(config);
    }

    public void setSolidColor(int r, int g, int b, int startIndex, int endIndex) {
        candle.setControl(
            new SolidColor(startIndex,endIndex)                
            .withColor(new RGBWColor(r, g, b, 0))
        );
    }

    public void clearLEDs() {
        setSolidColor(0, 0, 0, 0, totalLEDs-1);
    }
    
    @Override
    public void periodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();

        double fraction = Math.max(0.0, Math.min(1.0, (batteryVoltage - min) / (max - min))); //Clamp the voltage
        //so you can't get negative numbers, or numbers bigger than 12.8/max

        frontEndIndex = (int) (frontStrip * fraction);
        leftEndIndex = (int) (leftStrip * fraction);
        backEndIndex = (int) (backStrip * fraction);
        rightEndIndex = (int) (rightStrip * fraction);

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
        clearLEDs();

        if (frontEndIndex > 0) {
            setSolidColor(0, 0, 255, 8, 8 + frontEndIndex - 1);
        }

        if (backEndIndex > 0) {
            setSolidColor(0, 0, 255, 53, 53 + backEndIndex - 1);
        }
        if (leftEndIndex > 0) {
            setSolidColor(0, 0, 255, 72, 72 + leftEndIndex - 1);
        }
        if (rightEndIndex > 0) {
            setSolidColor(0, 0, 255, 91, 91 + rightEndIndex - 1);
        }
    }

    //Autonomous functions
    private void runAutoLEDs() {   
        clearLEDs();     
    }

    //Teleop functions
    private void runTeleopLEDs() {
        clearLEDs();
    }

}