import bluetooth
import struct
import time
from ble_advertising import advertising_payload
import machine
import utime
from micropython import const

toggle_state = 0
toggle_state2 = 0

button_20 = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP) # bottom button s3, handles range
button_22 = machine.Pin(22, machine.Pin.IN, machine.Pin.PULL_UP) # side button s2, handles intensity

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_READ | _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)

class BLESimplePeripheral:
    def __init__(self, ble, name="mpy-uart", allowed_mac=None):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._allowed_mac = allowed_mac
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[_UART_UUID])
        self._advertise()

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, addr_type, addr = data
            print("New connection from:", ':'.join('%02x' % i for i in addr))
            addr_str = ':'.join('%02x' % i for i in addr)  # Convert addr to string format

            if str(addr_str) != 'd8:3a:dd:bc:10:f5':
                print("Disconnecting: unwanted device", addr_str)
                self._ble.gap_disconnect(conn_handle)  # Disconnect if MAC doesn't match
                self._advertise()
                return
            print("Allowed device connected:", addr_str)
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)
            # Automatically restart advertising when disconnected
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            # Read the received value
            value = self._ble.gatts_read(value_handle)
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)

    def send(self, data):
        for conn_handle in self._connections:
            # Notify connected central device
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def is_connected(self):
        return len(self._connections) > 0

    def _advertise(self, interval_us=500000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def on_write(self, callback):
        self._write_callback = callback


# Define I2C pins
i2c = machine.I2C(0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400000)

# Define constants
TCA9548A_ADDRESS = 0x70
DRV2605L_ADDRESS = 0x5A

DRV2605_REG_STATUS = 0x00       #< Status register
DRV2605_REG_MODE = 0x01         #< Mode register
DRV2605_MODE_INTTRIG = 0x00     #< Internal trigger mode
DRV2605_MODE_REALTIME = 0x05    #< Real-time playback (RTP) mode
DRV2605_MODE_DIAGNOS = 0x06     #< Diagnostics mode
DRV2605_MODE_AUTOCAL = 0x07     #< Auto calibration mode
DRV2605_MODE_STANDBY = 0x40     #< Standby  mode

DRV2605_REG_RTPIN = 0x02    #< Real-time playback input register
DRV2605_REG_LIBRARY = 0x03  #< Waveform library selection register
DRV2605_REG_WAVESEQ1 = 0x04 #< Waveform sequence register 1
DRV2605_REG_WAVESEQ2 = 0x05 #< Waveform sequence register 2
DRV2605_REG_WAVESEQ3 = 0x06 #< Waveform sequence register 3
DRV2605_REG_WAVESEQ4 = 0x07 #< Waveform sequence register 4
DRV2605_REG_WAVESEQ5 = 0x08 #< Waveform sequence register 5
DRV2605_REG_WAVESEQ6 = 0x09 #< Waveform sequence register 6
DRV2605_REG_WAVESEQ7 = 0x0A #< Waveform sequence register 7
DRV2605_REG_WAVESEQ8 = 0x0B #< Waveform sequence register 8

DRV2605_REG_GO = 0x0C          #< Go register
DRV2605_REG_OVERDRIVE = 0x0D   #< Overdrive time offset register
DRV2605_REG_SUSTAINPOS = 0x0E  #< Sustain time offset, positive register
DRV2605_REG_SUSTAINNEG = 0x0F  #< Sustain time offset, negative register
DRV2605_REG_BREAK = 0x10       #< Brake time offset register
DRV2605_REG_AUDIOCTRL = 0x11   #< Audio-to-vibe control register
DRV2605_REG_AUDIOLVL = 0x12    #< Audio-to-vibe minimum input level register
DRV2605_REG_AUDIOMAX = 0x13    #< Audio-to-vibe maximum input level register
DRV2605_REG_AUDIOOUTMIN = 0x14 #< Audio-to-vibe minimum output drive register
DRV2605_REG_AUDIOOUTMAX = 0x15 #< Audio-to-vibe maximum output drive register
DRV2605_REG_RATEDV = 0x16      #< Rated voltage register
DRV2605_REG_CLAMPV = 0x17      #< Overdrive clamp voltage register
DRV2605_REG_AUTOCALCOMP = 0x18 #< Auto-calibration compensation result register
DRV2605_REG_AUTOCALEMP = 0x19  #< Auto-calibration back-EMF result register
DRV2605_REG_FEEDBACK = 0x1A   #< Feedback control register
DRV2605_REG_CONTROL1 = 0x1B   #< Control1 Register
DRV2605_REG_CONTROL2 = 0x1C   #< Control2 Register
DRV2605_REG_CONTROL3 = 0x1D   #< Control3 Register
DRV2605_REG_CONTROL4 = 0x1E   #< Control4 Register
DRV2605_REG_VBAT = 0x21       #< Vbat voltage-monitor register
DRV2605_REG_LRARESON = 0x22   #< LRA resonance-period register

# Define waveform index
waveform_index1 = 0x07  #7 softbumb 100% Example index, replace with your desired waveform index 0x01 is a single 100% click, 0x0C is a triple click
waveform_index2 = 0x0A  #10 doubleclick 100% Example index, replace with your desired waveform index 0x01 is a single 100% click, 0x0C is a triple click
waveform_index3 = 0x0C  #12 Triple Click 100% Example index, replace with your desired waveform index 0x01 is a single 100% click, 0x0C is a triple click

waveform_index4 = 0x08 # soft bumb 60%
waveform_index5 = 0x0B # double click 60%

# Function to select channel on multiplexer
def select_channel(channel):
    i2c.writeto(TCA9548A_ADDRESS, bytes([1 << channel]))

# Function to initialize DRV2605L on a specific channel
def init_drv2605L(channel):
    select_channel(channel)  # Select the channel where DRV2605L is connected
    # Calculate resonance period for 170Hz frequency
    resonance_period = int(1000 / 170)
    
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_MODE, bytes([0x00]))  # Set mode to internal trigger and deactivate auto-resonance
    
    # Configure the library selection and mode control registers
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_MODE_INTTRIG, bytes([0x00]))  # Internal trigger mode to use "GO trigger" in trigger_click method
    
    # Set the library selection to LRA library (6)
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_LIBRARY, bytes([0x06]))
    
    # Feedback control register
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_FEEDBACK, bytes([0xA4]))
    
    # Rated voltage for closed loop
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_RATEDV, bytes([0x68]))
    
    # Overdrive voltage for closed loop
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_CLAMPV, bytes([0x72]))
    
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_CONTROL1, bytes([0b10010111])) #see Control1 Register
    
    # LRA drive mode
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_CONTROL3, bytes([0x94])) # see Control3 Register in datsheet to make sharper response with 0xB4
    
    i2c.writeto_mem(DRV2605L_ADDRESS, 0x20, bytes([resonance_period]))
    
    # Write the waveform index to register 0x04 !! idk if correct
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_WAVESEQ1, bytes([waveform_index1]))  # Write waveform index to register 0x04
        # Write the waveform index to register 0x04 !! idk if correct
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_WAVESEQ1, bytes([waveform_index2]))  # Write waveform index to register 0x04
        # Write the waveform index to register 0x04 !! idk if correct
    i2c.writeto_mem(DRV2605L_ADDRESS, DRV2605_REG_WAVESEQ1, bytes([waveform_index3]))  # Write waveform index to register 0x04

def button_pressed_handler(pin):
    global toggle_state
    global toggle_state2

    # Compare the pin object with the global button objects
    if pin == button_20:
        toggle_state = 1 - toggle_state
        print(f"Button 20 pressed. Toggle state: {toggle_state}")
    elif pin == button_22:
        toggle_state2 = 1 - toggle_state2
        print(f"Button 22 pressed. Toggle state: {toggle_state2}")

    utime.sleep_ms(200)
    
def setup_buttons():
    # We no longer need a list of pin numbers, we have the global Pin objects
    buttons = {20: button_20, 22: button_22}
    for pin_number, button in buttons.items():
        # Configure the pin interrupt. Use IRQ_FALLING to detect when the button is pressed.
        button.irq(trigger=machine.Pin.IRQ_FALLING, handler=button_pressed_handler)
    return buttons

# Function to trigger click
def trigger_click(channel, pulse, toggle_state2):
    select_channel(channel)  
    if toggle_state2 == 0 and pulse == 1:
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index1]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))
    elif toggle_state2 == 0 and pulse == 2:
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index2]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))
    elif toggle_state2 == 0 and pulse == 3:
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index3]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))
    elif toggle_state2 == 1 and pulse == 1:
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index4]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))
    elif toggle_state2 == 1 and pulse == 2:
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index5]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))
    elif toggle_state2 == 1 and pulse == 3:
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index4]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index4]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x04, bytes([waveform_index4]))
        i2c.writeto_mem(DRV2605L_ADDRESS, 0x0C, bytes([0x01]))

    
# Main program
def main():
    global waveform_index1
    global waveform_index2
    global waveform_index3
    global waveform_index4
    global waveform_index5
    
    # Wait for at least 250 µs after power-up
    utime.sleep_us(250)
    
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)
    
    # Init led on pico
    led = machine.Pin("LED", machine.Pin.OUT)
    
    for channel in range(5):
        init_drv2605L(channel)  # Initialize all 5 DRV2605L
    
    # Set up buttons using the global Pin objects
    buttons = setup_buttons()
    
    # Enter your demo or main loop
    demo()
        
# Encryption and Decryption Key (8-bit)
encryption_key = 0xAB

# Function to decrypt data
def decrypt_data(encrypted_data):
    decrypted_data = bytearray()
    for byte in encrypted_data:
        decrypted_byte = byte ^ encryption_key
        decrypted_data.append(decrypted_byte)
    return decrypted_data

def demo():
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)
    def on_rx(v):
        global toggle_state
        global toggle_state2
        
        v= decrypt_data(v)
        message = int(v.decode().strip())  # Assuming the message is sent as a UTF-8 string
        print(message)
        chan = int((message) / 4)  # Convert to insteger
        pulse = message % 4
 
        if toggle_state == 1 and pulse == 3:
            # If toggle_state is 1 and pulse is 3, change pulse to 0.
            pulse = 0
            print(f"toggle_state is {toggle_state}, changing pulse to {pulse}")

        
        trigger_click(chan,pulse,toggle_state2)
        
    p.on_write(on_rx)

    while True:
        # This loop keeps the program running.
        
        time.sleep_ms(10)

if __name__ == "__main__":
    main()
