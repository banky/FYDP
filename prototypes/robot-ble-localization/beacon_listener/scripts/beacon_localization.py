import struct, time
from rospy import Time
from beacon_msgs.msg import LocationTag

import binascii


class BeaconLocalizationFilter:
    """Filters iterable of beacons to remove non-system and outdated ones,
        converts AdvertisingPacket instance to LocationTag msg"""

    def __init__(self, accepted_group, accepted_age):
        self.__accepted_group_id = accepted_group
        self.__accepted_age = accepted_age

    """Filter method, accepts iterable"""
    def do_filter(self, iterable):
        filtered = []
        for x in iterable:
            try:
                i = BeaconLocalizationAdvPacket(x)
                if i.gid != self.__accepted_group_id:
                    print 'pass in gid, expected: ' + format(self.__accepted_group_id, '02x') + ', given: ' + format(i.gid, '02x')
                    continue
                if time.time() - i.timestamp > self.__accepted_age:
                    continue
                filtered.append(i.to_msg())
            except PacketException:
                continue
        return filtered


class BeaconLocalizationAdvPacket:
    """Beacon Localization system advertising packet entity
    """

    def __init__(self, advertising_packet):
        # print "adv packet:"
        # print advertising_packet.scan_data
        # print len(advertising_packet.product)
        if len(advertising_packet.product) != 7: # "iBKS105"
            # print 'pass in size' + str(len(advertising_packet.scan_data[255]))
            # print binascii.hexlify(advertising_packet.scan_data[255])
            raise PacketException("Invalid size")
        # if struct.unpack('H', advertising_packet.scan_data[255][0:2])[0] != 0xFFFF:
            # print 'pass in code'
            # raise PacketException("Invalid manufacturer code")
        # self.gid = struct.unpack('I', advertising_packet.scan_data[255][2:6])[0]
        # self.bid = binascii.hexlify(advertising_packet.scan_data[255][6:12])
        
        self.gid = advertising_packet.gid
        self.bid = advertising_packet.bid
        self.rssi = advertising_packet.rssi
        self.timestamp = advertising_packet.updated_at

    """ Converts data to ROS LocationTag message (from beacon_msgs package)"""
    def to_msg(self):
        return LocationTag(self.gid, self.bid, self.rssi, Time.from_sec(self.timestamp))


class PacketException(Exception):
    def __init__(self, message):
        super(PacketException, self).__init__(message)
