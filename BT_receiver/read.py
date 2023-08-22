import bluetooth
import numpy as np
import matplotlib.pyplot as plt
import argparse
import json
from datetime import datetime

parser = argparse.ArgumentParser(
                    prog='read.py',
                    description='Script connecting with EMG device via Bluetoth and saving results to files. \
                        The device is sampling at constant frequency of 1 kHz.',
                    epilog='Script for research purposes only.')

parser.add_argument('measurement_time_s', metavar='T', type=int, nargs='?', default=5, 
                    help='Measurement time')
parser.add_argument('BT_addr', metavar='Addr', type=str, nargs='?', default="FC:F5:C4:19:6D:E2",
                    help='Bluetooth device address')
parser.add_argument('port', metavar='P', type=int, nargs='?', default=1,
                    help='Bluetooth device port')

args = parser.parse_args()

measurement_time_s = args.measurement_time_s
bd_addr = args.BT_addr
port = args.port

sampling_freq = 1000 #Hz
channels = 3
bytes_per_value = 2
values_to_receive = measurement_time_s*sampling_freq
received_data = np.empty((channels, values_to_receive))

# Create a Bluetooth socket and connect to the device
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr, port))

for i in range(values_to_receive):
    for channel in range(channels):
        data = sock.recv(bytes_per_value)
        if not data:
            pass
        received_data[channel][i] = int.from_bytes(data, byteorder='big')
        print(received_data[channel][i])


# Close the socket
sock.close()

data_dict = {
    "measurement_time_s" : measurement_time_s,
    "sampling_freq" : sampling_freq,
    "channels": channels,
    "channel1": received_data[0],
    "channel2": received_data[1],
    "channel3": received_data[2]
}

with open('output/EMG_'+ datetime.now().strftime("%d%m%Y_%H%M%S") +'.json', 'w') as fp:
    json.dump(data_dict, fp, indent=3)

# Plot the received data for each channel
for channel in range(channels):
    plt.plot(received_data[channel])
    plt.title('Channel {}'.format(channel + 1))
    plt.xlabel('Sample index')
    plt.ylabel('Value')
    plt.show()