import threading
import time
import logging
from jnius import autoclass, cast

logger = logging.getLogger(__name__)
serial_lock = threading.Lock()

PythonActivity = autoclass("org.kivy.android.PythonActivity")
UsbManager = autoclass("android.hardware.usb.UsbManager")
UsbDeviceConnection = autoclass("android.hardware.usb.UsbDeviceConnection")
UsbSerialDevice = autoclass("com.felhr.usbserial.UsbSerialDevice")
UsbSerialInterface = autoclass("com.felhr.usbserial.UsbSerialInterface")
PendingIntent = autoclass("android.app.PendingIntent")
Intent = autoclass("android.content.Intent")
Context = autoclass("android.content.Context")


class ArduinoConnection:
    def __init__(self, vendor_id=None, product_id=None, baudrate=115200, max_retries=3, reboot_wait=2.0):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.baudrate = baudrate
        self.max_retries = max_retries
        self.reboot_wait = reboot_wait

        self.usb_manager = cast("android.hardware.usb.UsbManager",
                                PythonActivity.mActivity.getSystemService(Context.USB_SERVICE))
        self.device = None
        self.connection = None
        self.serial_port = None

        self.init_usb_connection()

    def init_usb_connection(self):
        try:
            device_list = self.usb_manager.getDeviceList().values().toArray()
            for d in device_list:
                if (self.vendor_id is None or d.getVendorId() == self.vendor_id) and \
                   (self.product_id is None or d.getProductId() == self.product_id):
                    self.device = d
                    break

            if not self.device:
                logger.error("Arduino USB device not found")
                return False

            intent = PendingIntent.getBroadcast(PythonActivity.mActivity, 0,
                                               Intent("com.example.USB_PERMISSION"), 0)
            self.usb_manager.requestPermission(self.device, intent)

            time.sleep(self.reboot_wait)

            self.connection = self.usb_manager.openDevice(self.device)
            if not self.connection:
                logger.error("Failed to open USB device")
                return False

            self.serial_port = UsbSerialDevice.createUsbSerialDevice(self.device, self.connection)
            if self.serial_port is None:
                logger.error("Failed to create UsbSerialDevice")
                return False

            if not self.serial_port.open():
                logger.error("Failed to open serial port")
                return False

            self.serial_port.setBaudRate(self.baudrate)
            self.serial_port.setDataBits(UsbSerialInterface.DATA_BITS_8)
            self.serial_port.setStopBits(UsbSerialInterface.STOP_BITS_1)
            self.serial_port.setParity(UsbSerialInterface.PARITY_NONE)
            self.serial_port.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF)

            logger.info("USB Serial connection established")
            return True

        except Exception as e:
            logger.error(f"Error initializing USB Serial connection: {e}")
            self.connection = None
            self.serial_port = None
            return False

    def send_command(self, command):
        if isinstance(command, str):
            command = command.encode()

        for _ in range(self.max_retries):
            try:
                with serial_lock:
                    if not self.serial_port or not self.serial_port.isOpen():
                        if not self.init_usb_connection():
                            time.sleep(0.1)
                            continue

                    self.serial_port.write(command)
                    return True
            except Exception as e:
                logger.warning(f"Send failed, retrying: {e}")
                try:
                    self.init_usb_connection()
                except:
                    pass
                time.sleep(0.1)
        return False

    def close(self):
        try:
            if self.serial_port:
                self.serial_port.close()
            if self.connection:
                self.connection.close()
            self.serial_port = None
            self.connection = None
            logger.info("USB Serial connection closed")
        except Exception as e:
            logger.error(f"Error closing USB Serial: {e}")
