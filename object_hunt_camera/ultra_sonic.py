import socket
import sys
import argparse
import struct

BUFFER_SIZE = 256

parser = argparse.ArgumentParser(description='Test socket commands for the ultra sonic sensors.')

parser.add_argument('-i', dest='id', type=int, default=1, help='The identifier for this measurement. Identifier have to be ascending.')
parser.add_argument('-d', dest='device', type=int, default=5, help='The ultra sonic sensor that will carry out the measurement.')

args = parser.parse_args()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 19001)
print >>sys.stderr, 'connecting to %s port %s' % server_address
sock.connect(server_address)

try:

    # Send data
    # message = (0x00 << 40) | (args.id << 8) | args.device
    print >>sys.stderr, 'sending the message'
    sock.send(struct.pack('<B', 0))
    sock.send(struct.pack('<IB', args.id, args.device))

    # Wait for the response
    response = sock.recv(1)
    if struct.unpack('<B', response) is 1:
    found

    print('Received: Identifier: {}, Distance: {}'.format(response_id, response_distance))


finally:
    print >>sys.stderr, 'closing socket'
    sock.close()
