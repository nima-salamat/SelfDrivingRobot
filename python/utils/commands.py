import threading
import time
import logging
from jnius import autoclass, cast

logger = logging.getLogger(__name__)
serial_lock = threading.Lock()

UsbManager = autoclass('android.hardware.usb.UsbManager')
UsbDeviceConnection = autoclass('android.hardware.usb.UsbDeviceConnection')
UsbDevice = autoclass('android.hardware.usb.UsbDevice')
PendingIntent = autoclass('android.app.PendingIntent')
IntentFilter = autoclass('android.content.IntentFilter')
Context = autoclass('android.content.Context')
PythonActivity = autoclass('org.kivy.android.PythonActivity')


class ArduinoConnection:
    def __init__(self, vendor_id=None, product_id=None, max_retries=3, reboot_wait=2.0):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.max_retries = max_retries
        self.reboot_wait = reboot_wait
        self.usb_manager = cast('android.hardware.usb.UsbManager', PythonActivity.mActivity.getSystemService(Context.USB_SERVICE))
        self.device = None
        self.connection = None
        self.init_usb_connection()

    def init_usb_connection(self):
        try:
            devices = self.usb_manager.getDeviceList().values().toArray()
            for d in devices:
                if (self.vendor_id is None or d.getVendorId() == self.vendor_id) and \
                   (self.product_id is None or d.getProductId() == self.product_id):
                    self.device = d
                    break

            if not self.device:
                logger.error("Arduino USB device not found")
                return False

            intent = PendingIntent.getBroadcast(PythonActivity.mActivity, 0, 
                                               autoclass('android.content.Intent')('com.example.USB_PERMISSION'), 0)
            self.usb_manager.requestPermission(self.device, intent)
            time.sleep(self.reboot_wait)
            self.connection = self.usb_manager.openDevice(self.device)
            if self.connection:
                logger.info("USB connection established")
                return True
            else:
                logger.error("Failed to open USB device")
                return False
        except Exception as e:
            logger.error(f"Error initializing USB connection: {e}")
            self.connection = None
            return False

    def send_command(self, command):
        if isinstance(command, str):
            command = command.encode()

        for _ in range(self.max_retries):
            try:
                with serial_lock:
                    if not self.connection:
                        if not self.init_usb_connection():
                            time.sleep(0.1)
                            continue

               
                    endpoint_out = self.device.getInterface(0).getEndpoint(0)
                    self.connection.bulkTransfer(endpoint_out, command, len(command), 1000)
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
        if self.connection:
            try:
                self.connection.close()
                self.connection = None
                logger.info("USB connection closed")
            except Exception as e:
                logger.error(f"Error closing USB connection: {e}")
