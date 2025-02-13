package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;


// NOTE: THIS CLASS MAY NOT WORK INDEPENDENTLY BECAUSE THE REQUIRED SUBSYSTEMS ARE INVISIBLE TO IT 
public class LEDController implements Subsystem {
    // DECLARATIONS
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private double cycleTicks, totalTicks;

    private TargetRGB currentRGB;
    private boolean blink;

    private ElevatorSubsystem elevator;
    private PivotSubsystem pivot;

    private IntakeSubsystem intake;
    private WristSubsystem wrist;

    /**LED Constructor is set here
     * @param port
     * Location (port) set for LED strips
     * @param ledLength
     * The total length of LED strips
     */
    public LEDController(
        PivotSubsystem pivot, ElevatorSubsystem elevator, WristSubsystem wrist) {
        // Constructor for LED objects
        leds = new AddressableLED(LEDConstants.kPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.kLED_Length);

        // Sets LED length to leds
        leds.setLength(LEDConstants.kPort);

        cycleTicks = 0;

        // SUBSYSTEMS TO USE FOR CONDITIONALS
        elevator = new ElevatorSubsystem();
        pivot = new PivotSubsystem();
        wrist = new WristSubsystem();
        currentRGB = TargetRGB.NONE;
    }
    /**Changes the colot of LEDs
     * @param targetRGB
     * Wanted RGB color
     */
    public void setColor(TargetRGB targetRGB) {
        // STOPS FROM WASTING RESOURCES VIA RETURN
        if(currentRGB.equals(targetRGB)) return;
        // ALLOWS LEDS TO TURN ON/OFF, BLINK FOR INTAKE
        if(blink && totalTicks % 50 <= 25) {
            setLedBuffer(TargetRGB.NONE);
        } else {
            setLedBuffer(targetRGB);
        }
        leds.setData(ledBuffer);
    }
    /**sets the RGB of entire LED strip defined in ledBuffer
     * @param targetRGB 
     * Uses stored enum data to assign LED RGB
     */
    public void setLedBuffer(TargetRGB targetRGB) {
        for (int ledSquares = 0; ledSquares < ledBuffer.getLength(); ledSquares++) {
            ledBuffer.setRGB(ledSquares, targetRGB.red, targetRGB.blue, targetRGB.green);
        }
    }
    // METHODS TO MANIPULATE LEDs in-match
    /**Enables LEDs and assigns color to "START_UP" Enum */
    public void start() {
        leds.start();
    }
    /**Disables LEDs*/
    public void stop() {
        leds.stop();
    }
    /**Runs periodically, uses conditionals to change LED color */
    @Override
    public void periodic() {
        // TODO USE KNOWN DATA FROM EACH SUBSYSTEM AND MAKE A DECISION FOR THE LED STATE BASED ON ALL OPF THE SUBSYSTEMS\
        cycleTicks++;
        totalTicks++;
        if(totalTicks > 50000) {
            totalTicks = 0;
        }
        if(cycleTicks >= 10) {
            if(!elevator.isAtSetpoint() 
            || !pivot.isAtSetpoint()
            || !wrist.rotateIsAtSetpoint()
            || !wrist.tiltIsAtSetpoint()) {
                setColor(TargetRGB.IN_TRANSITION);
            } else {
                if(elevator.currentTargetPosition != ElevatorPosition.STORED) {
                    setColor(TargetRGB.AT_POSITION);
                } else {
                    setColor(TargetRGB.BASE);
                }
                if(intake.isRunning()) {
                    blink = true;
                } else {
                    blink = false;
                }
            } 
            cycleTicks = 0;
        }
    }

    public Command untilAtPosition() {
        return new WaitUntilCommand(() -> {
            setColor(TargetRGB.IN_TRANSITION);
            // PIVOT IS NOT INCLUDED BECAUSE IT IIRST TO MOVE
            return !elevator.isAtSetpoint()
            || !wrist.rotateIsAtSetpoint()
            || !wrist.tiltIsAtSetpoint();
        });
    }

    /**enums for determining RGBs of LEDs*/
    public enum TargetRGB {
        NONE(0, 0, 0),

        BASE(18, 148, 255), // Blue

        IN_TRANSITION(255, 13, 13),// Red

        OBTAINED_CORAL(240, 200, 0),// Gold

        IS_PECKING(255,255,255),// White

        AT_POSITION(49,255, 13),//Green

        INTERUPTED(-1,-1,-1);

        // THE Following are maybes to use

        // IS_PECKING(-1,-1,-1),

        
        // VARIABLES TO SET INTS for LEDs' enums
        int red;
        int green;
        int blue;

        /**Constructor for LEDs color
         * @param hue
         * Determines hue of LEDs
         * @param saturation
         * determines saturation of LEDs
         * @param value
         * determines value of LEDs
         */
        private TargetRGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
}