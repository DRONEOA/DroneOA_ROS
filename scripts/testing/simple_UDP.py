import socket
import sys
from pymavlink import fgFDM

udp = socket.socket(socket.AF_INET, # Internet
                socket.SOCK_DGRAM) # UDP
udpSender = socket.socket(socket.AF_INET, # Internet
                socket.SOCK_DGRAM) # UDP
ue4_address = ('127.0.0.1', 23339)
fdm = fgFDM.fgFDM()

def init():
    # Create a UDP socket, careful not use ports occupied by SITL
    sitl_address = ('127.0.0.1', 5503)
    udp.bind(sitl_address)

def tryRead():
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
        printLineMsg = "GPS: %s %s %s SPD: %s PRY: %s %s %s" % (latitude, longitude, altitude, vcas, pitch, roll, yaw)
        # print(printLineMsg)
        sent = udpSender.sendto(printLineMsg, ue4_address)

def closeUDP():
    print >>sys.stderr, 'closing socket'
    udp.close()
    udpSender.close()

if __name__=="__main__":
    init()
    tryRead()
    closeUDP()
