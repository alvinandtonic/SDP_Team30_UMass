import serial
import time
from bluepy import btle

# Encryption and Decryption Key (8-bit)
encryption_key = 0xAB

# Function to encrypt data
def encrypt_data(data):
    encrypted_data = bytearray()
    for byte in data:
        encrypted_byte = byte ^ encryption_key
        encrypted_data.append(encrypted_byte)
    return encrypted_data


class BLEClient(object):

    def __init__(self, mac_address):
        self.mac_address = mac_address
        self.device = None
        self.connect()

    def connect(self):
        print(f"Connecting to {self.mac_address}")
        self.device = btle.Peripheral(self.mac_address)
        print("Connected")

    def write_to_characteristic(self, service_uuid, char_uuid, message):
        # Convert message to bytes if it's a string
        if isinstance(message, str):
            message = message.encode('utf-8')
        # Encrypt the message
        message = encrypt_data(message)
        # Proceed with writing to the characteristic
        service = self.device.getServiceByUUID(service_uuid)
        characteristic = service.getCharacteristics(char_uuid)[0]
        characteristic.write(message)

    def disconnect(self):
        if self.device:
            self.device.disconnect()
            print("Disconnected")

# Specific ID of the device to connect
service_uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
char_uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
mac_address = "28:CD:C1:0A:10:5E"

# Baud Rate for Cyglidar
baud = 250000

# Command bytes for starting and stopping 3D data acquisition
RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x08, 0x00, 0x0A]
#RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x12, 0x39, 0x29] #lowest rate 57600
#RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x12, 0xAA, 0xBA] #low rate 115200
#RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x12, 0x77, 0x67] #high rate 250000
#RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x12, 0x55, 0x45] #high rate 3000000
COMMAND_STOP = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x02, 0x00, 0x00]

# Index constants for parsing received data
HEADER1, HEADER2, HEADER3, LENGTH_LSB, LENGTH_MSB, PAYLOAD_HEADER, PAYLOAD_DATA, CHECKSUM = 0, 1, 2, 3, 4, 5, 6, 7

# Constants for device identification and data normalization
NORMAL_MODE = 0x5A
PRODUCT_CODE = 0x77
DEFAULT_ID = 0xFF
dataLength3D = 14400

# Serial communication settings
ser = serial.Serial(
    port="/dev/ttyUSB0",  # Port configuration for USB connection
    baudrate=baud,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.1  # Set a timeout for non-blocking read
)

# Function to handle complete received data
def ReceivedCompleteData(receivedData):
    global dataLength3D

    if len(receivedData) == dataLength3D:
        Rawdata = Get3DDistanceData(receivedData)
        CompartedData = CompartmentalizeData(Rawdata)
        FivePointData = AbridgeData(CompartedData)
        print(FivePointData)
        SignalThreshold(FivePointData)


# Function to extract 3D distance data from received data
def Get3DDistanceData(receivedData):
    global dataLength3D
    index = 0
    distanceData = [0 for _ in range(int(dataLength3D / 3 * 2))]
    for i in range(0, dataLength3D - 2, 3):
        pixelFirst = receivedData[i] << 4 | receivedData[i + 1] >> 4
        pixelSecond = (receivedData[i + 1] & 0xf) << 8 | receivedData[i + 2]

        distanceData[index] = int(pixelFirst / 20)
        index += 1
        distanceData[index] = int(pixelSecond / 20)
        index += 1
    return distanceData

# Function to seperate data in 2d plane matrix to 5 different angle group arrays
def CompartmentalizeData(Rawdata):
    CompartedData = []
    if len(Rawdata) == 9600:
        for cursor in range(0, len(Rawdata), 160):
            DataPortion = []
            for cursor2 in range(cursor, cursor + 160):
                DataPortion.append(Rawdata[cursor2])
            CompartedData.append(DataPortion)
        return CompartedData

# Function to calculate data in each angle group into a single data to be processed for threshold
def AbridgeData(CompartedData):
    abridged = []
    for cursor in range(0, 160, 32):
        average1 = 300
        average2 = 300
        average3 = 300
        for cursor2 in range(0, 50):
            for cursor3 in range(cursor, cursor + 32):
                if average1 > CompartedData[cursor2][cursor3]:
                    average3 = average2
                    average2 = average1
                    average1 = CompartedData[cursor2][cursor3]
                    continue
                elif average2 >= CompartedData[cursor2][cursor3]:
                    average3 = average2
                    average2 = CompartedData[cursor2][cursor3]
                    continue
                elif average3 >= CompartedData[cursor2][cursor3]:
                    average3 = CompartedData[cursor2][cursor3]
                    continue
                else:
                    continue
                    
        abridged.append((average1+average2+average3)/3)
    return abridged

# Function to signal to slave device 
def SignalThreshold(FivePointData):
    for cursor in range(0, 5):
        if 150 < FivePointData[cursor]:
            devicen= cursor*4+0
            client.write_to_characteristic(service_uuid, char_uuid, str(devicen))
        elif 150 > FivePointData[cursor] > 75:
            devicen= cursor*4+1
            client.write_to_characteristic(service_uuid, char_uuid, str(devicen))
        elif 75 > FivePointData[cursor] > 25:
            devicen= cursor*4+2
            client.write_to_characteristic(service_uuid, char_uuid, str(devicen))
        elif FivePointData[cursor] < 25:
            devicen= cursor*4+3
            client.write_to_characteristic(service_uuid, char_uuid, str(devicen))


# Main function
if __name__ == "__main__":
    # Initialization
    time.sleep(1)
    client = BLEClient(mac_address)
    ser.write(RUN_3D)
    print("send: ", RUN_3D)
    step = HEADER1
    CPC = 0

    bufferCounter = 0
    receivedData = [0 for _ in range(dataLength3D)]
    
    # Main Loop
    while True:
        try:
            
            # Read multiple bytes at once
            data = ser.read(1024)  

            for byte in data:
                parserPassed = False

                # Parse Start
                if step != CHECKSUM:
                    CPC = CPC ^ byte
                if step == PAYLOAD_DATA:
                    receivedData[bufferCounter] = byte
                    bufferCounter += 1
                    if bufferCounter >= dataLength:
                        step = CHECKSUM
                elif step == HEADER1 and byte == NORMAL_MODE:
                    step = HEADER2
                elif step == HEADER2 and byte == PRODUCT_CODE:
                    step = HEADER3
                elif step == HEADER3 and byte == DEFAULT_ID:
                    step = LENGTH_LSB
                    CPC = 0
                elif step == LENGTH_LSB:
                    step = LENGTH_MSB
                    lengthLSB = byte
                elif step == LENGTH_MSB:
                    step = PAYLOAD_HEADER
                    lengthMSB = byte
                    dataLength = (lengthMSB << 8) | lengthLSB - 1
                elif step == PAYLOAD_HEADER:
                    step = PAYLOAD_DATA
                    if dataLength == 0:
                        step = CHECKSUM
                    bufferCounter = 0
                    receivedData = [0 for _ in range(dataLength)]  # clear
                elif step == CHECKSUM:
                    step = HEADER1
                    if CPC == byte:
                        parserPassed = True
                else:
                    step = HEADER1
                    parserPassed = False

                # Parse End

                if parserPassed:
                    ReceivedCompleteData(receivedData)
        except KeyboardInterrupt:
            client.disconnect()
            ser.write(COMMAND_STOP)
            ser.close()

#pkill -9 -f script.py kills process