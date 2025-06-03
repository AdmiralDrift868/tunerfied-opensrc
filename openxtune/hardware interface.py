# core/connection_manager.py
import can
from can.interfaces import socketcan

class ConnectionManager:
    def __init__(self):
        self.bus = None
        self.protocol = None
        self.connected = False
        
    def connect(self, interface='socketcan', channel='can0', protocol='obd2'):
        """Connect to vehicle bus"""
        try:
            self.bus = can.interface.Bus(channel=channel, bustype=interface)
            self.protocol = self._get_protocol(protocol)
            self.connected = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
            
    def _get_protocol(self, protocol):
        """Load appropriate protocol handler"""
        protocols = {
            'obd1': OBD1Protocol,
            'obd2': OBD2Protocol,
            'can': CANProtocol,
            'j1939': J1939Protocol
        }
        return protocols.get(protocol.lower(), OBD2Protocol)()
        
    def read_map(self, map_name):
        """Read specific map from ECU"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        return self.protocol.read_map(map_name)
        
    def write_map(self, map_data):
        """Write map to ECU"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        self.protocol.write_map(map_data)