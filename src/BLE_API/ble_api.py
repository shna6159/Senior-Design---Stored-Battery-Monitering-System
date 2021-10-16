#!/usr/bin/env python3.7
from blatann import BleDevice


def ble_api_device_setup():
    print("Setting up the device")
    ble_device = BleDevice("COM5")
    ble_device.configure()
    ble_device.open()
    return ble_device
    # Ready to use

def ble_api_scan(ble_device):
    scan_report_collection = ble_device.scanner.start_scan().wait(timeout=20)
    for report in ble_device.scanner.start_scan().scan_reports:
        if not report.duplicate:
            if "90:F1:57" in report.device_name:
                print("Found!!!")
                print("Mac address : {0}, RSSI[dBm] : {1}".format(report.device_name,report.rssi))

def main():
    ble_device = ble_api_device_setup()
    ble_api_scan(ble_device)


if __name__ == "__main__":
    main()