# pylint: disable=C0103
# pylint: disable=C0325

"""
Translate & Forward data from SITL to UE4 Simulator via UDP
"""

import socket
import six
from pymavlink import fgFDM # pylint: disable=E0401

udp = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
udpSender = socket.socket(socket.AF_INET, # Internet
                          socket.SOCK_DGRAM) # UDP
ue4_address = ('127.0.0.1', 23339)
fdm = fgFDM.fgFDM()

def init():
    """
    Initialize the UDP socket (Listener & Sender)
    """
    # Create a UDP socket, careful not use ports occupied by SITL
    sitl_address = ('127.0.0.1', 5503)
    udp.bind(sitl_address)

def try_read():
    """
    Read data from SITL and forward to UE4 SIM
    """
    # Receive response
    while True:
        data = udp.recv(4096)
        fdm.parse(data)
        latitude = fdm.get('latitude', units='degrees')
        longitude = fdm.get('longitude', units='degrees')
        altitude = fdm.get('altitude', units='meters')
        vcas = fdm.get('vcas', units='mps')
        pitch = fdm.get('theta', units='radians')
        roll = fdm.get('phi', units='radians')
        yaw = fdm.get('psi', units='radians')
        rpm = fdm.get('rpm', 0)
        print_line_msg = "GPS: %s %s %s SPD: %s PRY: %s %s %s RPM: %s" % ( \
            latitude, longitude, altitude, vcas, pitch, roll, yaw, rpm)
        # print(print_line_msg)
        sent = udpSender.sendto(six.b(print_line_msg), ue4_address)  # pylint: disable=W0612
        # print(send)

def close_udp():
    """
    Close all UDP socket
    """
    print('closing socket')
    udp.close()
    udpSender.close()

if __name__ == "__main__":
    init()
    try_read()
    close_udp()
