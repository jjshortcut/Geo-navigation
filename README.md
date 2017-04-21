# Geo-navigation
Bike-Nav and Geocache device

Welcome to the Geo-navigation wiki!

**HM-10 BLE module settings:**
- Baud = 9600

Characteristic of HM-10 are:
- Service UUID: 0xFFE0 (Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb)
- Characteristic: 0xFFE1 (Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb)

The HM-10 abstracts and packs a Bluetooth Low Energy connection in a serial connection. The original out-of-the-box firmware of the module exposes a BLE peripheral with a proprietary connectivity service (Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb) that enables bidirectional communication between the module and any other central device that connects to it. The service defines a single characteristic (Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb) that stores 20 bytes of unformatted data: 
When the central device wants to send data to the module, it WRITES the charactreristic with the desired content 
When the module wants to send data, it sends a NOTIFICATION to the central device

http://blog.blecentral.com/2015/05/05/hm-10-peripheral/
