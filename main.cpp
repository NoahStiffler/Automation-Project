#include "ClearCore.h"
#define INPUT_A_B_FILTER 20
#define motor ConnectorM2
#define HomingSensor ConnectorDI7
#define InputConnector ConnectorDI6

#define baudRate 9600

#define SerialPort ConnectorUsb

void HomingSensorCallback();
bool MoveToPosition(uint8_t positionNum);
bool risen;

int main() {
    // This section attaches the interrupt callback to the homing sensor pin,
    // set to trigger on any change of sensor state. (yellow button)
    HomingSensor.Mode(Connector::INPUT_DIGITAL);
    HomingSensor.InterruptHandlerSet(HomingSensorCallback, InputManager::CHANGE);
    //Absolute Position mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_A_DIRECT_B_DIRECT);

    motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);

    motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    
    motor.MotorInAState(false);
    // Set input B to match the initial state of the sensor.
    motor.MotorInBState(HomingSensor.State());
//	Delay_ms(2000);
//	MoveToPosition(1);
    // Sets up serial communication and waits up to 5 seconds for a port to open
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    SerialPort.PortOpen();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        continue;
    }
    // Enables the motor and starts homing
    motor.EnableRequest(true);
    SerialPort.SendLine("Motor Enabled");
    // Wait for HLFB
    SerialPort.SendLine("Waiting for HLFB...");
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    SerialPort.SendLine("Motor Ready");
	MoveToPosition(2);
    while (true) {
		risen = InputConnector.InputRisen();
		if (risen) {
        MoveToPosition(1);
		Delay_ms(1500);
		MoveToPosition(2);    
		}
    }
}

bool MoveToPosition(uint8_t positionNum) {
    // Check if an alert is currently preventing motion
    if (motor.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor status: 'In Alert'. Move Canceled.");
        return false;
    }
    SerialPort.Send("Moving to position: ");
    SerialPort.Send(positionNum);
    switch (positionNum) {
        case 1:
            // Sets Input A "off" for position 1
            motor.MotorInAState(false);
            SerialPort.SendLine(" (Input A Off)");
            break;
        case 2:
            // Sets Input A "on" for position 2
            motor.MotorInAState(true);
            SerialPort.SendLine(" (Input A On)");
            break;
        default:
            // If this case is reached then an incorrect positionNum was entered
            return false;
    }

    Delay_ms(2 + INPUT_A_B_FILTER);
    // Wait for HLFB 
    SerialPort.SendLine("Moving... Waiting for HLFB");
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    SerialPort.SendLine("Move Done");
    return true;
}

void HomingSensorCallback() {
    Delay_ms(1);
    motor.MotorInBState(HomingSensor.State());
}