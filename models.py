from pymodbus.client.sync import ModbusTcpClient

class Scara:
    def __init__(self, **kwargs):
        # they are all modbus clients
        self.J1 = kwargs.get('J1', None)
        self.J2 = kwargs.get('J2', None)
        self.J3 = kwargs.get('J3', None)
        self.J4 = kwargs.get('J4', None)

    def move(self, **kwargs):
        pass