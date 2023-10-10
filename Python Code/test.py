import usb.core
import usb.backend.libusb1

backend = usb.backend.libusb1.get_backend(find_library=lambda x: "/path/to/libusb-1.0.so")
dev = usb.core.find(backend=backend)
print(dev)
