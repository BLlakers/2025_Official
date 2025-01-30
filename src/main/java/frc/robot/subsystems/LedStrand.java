package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrand extends SubsystemBase {
    private SerialPort arduinoSerial;

    public LedStrand() {
        try {
            arduinoSerial = new SerialPort(9600, SerialPort.Port.kUSB1); // Try USB1 first
        } catch (Exception e1) {
            System.out.println("Couldn't connect to USB1, trying USB2...");
            try {
                arduinoSerial = new SerialPort(9600, SerialPort.Port.kUSB2);
            } catch (Exception e2) {
                System.out.println("Couldn't connect to USB2. Disabling LED communication.");
                arduinoSerial = null; // Prevent null pointer issues
            }
        }
    }

    public void stopLed() {
        sendSerialCommand("R0G0B0");
    }

    public void changeLed(int r, int g, int b) {
        sendSerialCommand("R"+r+"G"+r+"B"+b);
    }

    public Command changeLedCommand(int r, int g, int b) {
        return this.runOnce(() -> changeLed(r, g, b));
    }

    private void sendSerialCommand(String command) {
        if (arduinoSerial != null) {
            arduinoSerial.writeString(command);
        } else {
            System.out.println("Serial communication unavailable. Cannot send: " + command);
        }
    }
}
