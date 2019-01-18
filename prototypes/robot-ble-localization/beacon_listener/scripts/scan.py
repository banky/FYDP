from bluepy.btle import DefaultDelegate
import time
import copy
import rospy

class AdvertisingPacket:
    """Beacon entity"""

    def __init__(self, scan_entry):
        self.addr = scan_entry.addr
        self.updated_at = time.time()
        self.raw = scan_entry.rawData
        self.rssi = scan_entry.rssi
        # self.scan_data = copy.deepcopy(scan_entry.scanData)
        self.scan_data = scan_entry.getScanData()
        self.product = ""
        self.gid = 0
        self.bid = 0


class BeaconScanDelegate(DefaultDelegate):
    """Delegate to handle bluepy Scanner events"""

    def __init__(self, container=None):
        DefaultDelegate.__init__(self)
        self.__container = container

    def set_container(self, container):
        self.__container = container

    def handleDiscovery(self, scanEntry, isNewDev, isNewData):
        if isNewDev:
            rospy.loginfo("New Device Found:")
            rospy.loginfo("Discovery of device %s, RSSI: %s", scanEntry.addr, scanEntry.rssi)
            pass

        if scanEntry.addr == "e2:f4:c9:c1:fa:29": # Beacon A
            rospy.loginfo("Discovery of device %s, RSSI: %s", scanEntry.addr, scanEntry.rssi)
            packet = AdvertisingPacket(scanEntry)
            packet.product = scanEntry.getValueText(9) # Get the product name

            if packet.product is None:
                return
            
            packet.gid = 0x11111111
            packet.bid = "aa5aab45bdcf"
            self.__container.insert(packet.addr, packet)

        elif scanEntry.addr == "f4:69:bc:c3:91:f6": # Beacon B
            rospy.loginfo("Discovery of device %s, RSSI: %s", scanEntry.addr, scanEntry.rssi)
            packet = AdvertisingPacket(scanEntry)
            packet.product = scanEntry.getValueText(9) # Get the product name

            if packet.product is None:
                return
            
            packet.gid = 0x11111111
            packet.bid = "237b44757adc"
            self.__container.insert(packet.addr, packet)

        elif scanEntry.addr == "f5:42:9c:3f:66:ac": # Beacon C
            rospy.loginfo("Discovery of device %s, RSSI: %s", scanEntry.addr, scanEntry.rssi)
            packet = AdvertisingPacket(scanEntry)
            packet.product = scanEntry.getValueText(9) # Get the product name
            

            if packet.product is None:
                return

            packet.gid = 0x11111111
            packet.bid = "aaa25624c9e4"
            self.__container.insert(packet.addr, packet)
