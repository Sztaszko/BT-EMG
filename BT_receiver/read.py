import bluetooth
import numpy as np
import matplotlib.pyplot as plt

measurement_time_s = 5
sampling_freq = 1000 #Hz
channels = 3
bytes_per_value = 2
values_to_receive = measurement_time_s*sampling_freq
received_data = np.empty((channels, values_to_receive))
# Define the Bluetooth device address and port number
bd_addr = "FC:F5:C4:19:6D:E2"
port = 1 

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


# Plot the received data for each channel
for channel in range(channels):
    plt.plot(received_data[channel])
    plt.title('Channel {}'.format(channel + 1))
    plt.xlabel('Sample index')
    plt.ylabel('Value')
    plt.show()