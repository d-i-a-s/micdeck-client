import numpy as np

class StreamPort:

    def __init__(self, crazyflie, q, bytesOfData, CF_ON_TIME, SAMPLING_FREQ):
        # Initializes different values
        self.bytesOfData = bytesOfData
        # Last received data buffer
        self.newData = np.zeros(self.bytesOfData, dtype=np.uint8)
        # Previous to last received data buffer
        self.dispData = np.zeros(self.bytesOfData, dtype=np.uint8)
        # Vector to which the data in unpacked
        self.unpackedData = np.zeros(19, dtype=np.uint16)
        # Flags if any packet was received
        self.packetRecieved = 0
        # Imported queue from continousStream
        self.q = q
        # Packet count
        self.packetCountLoop = 0
        # Packet lost count
        self.packetLostCount = 0
        # Crazyflie pointer
        self.cf = crazyflie
        # Adds callback to respective port
        self.cf.add_port_callback(0x01, self.incoming)
        # Audio vector
        self.audioVector = np.zeros(CF_ON_TIME*SAMPLING_FREQ, dtype=np.int32)
        self.ptr = 0;

    def unpack_stream(self):
        aux = np.uint16
        self.unpackedData = np.zeros(19, dtype=np.uint16)
        ptr = 0
        halfPtr = False

        for i in range(0, 19):
            aux = 0
            if not halfPtr:
                aux = self.dispData[ptr]
                aux = aux << 4
                ptr += 1
                aux = aux | (self.dispData[ptr] >> 4)
                self.unpackedData[i] = aux
                halfPtr = True
            else:
                aux = self.dispData[ptr] & 0x0F
                aux = aux << 8
                ptr += 1
                aux = aux | self.dispData[ptr]
                ptr += 1
                self.unpackedData[i] = aux
                halfPtr = False

    def incoming(self, packet):
        # Callback for data received from the copter.
        # If is the first packet received buffers it
        if not self.packetRecieved:
            self.newDataPacketCount = packet.data[0]
            self.newData = packet.data[1:]
            self.packetRecieved += 1
        else:
            self.dispDataPacketCount = self.newDataPacketCount
            self.dispData = self.newData
            self.newDataPacketCount = packet.data[0]
            self.newData = packet.data[1:]

            self.unpack_stream()
            self.packetRecieved += 1
            if self.newDataPacketCount > self.dispDataPacketCount:
                jump = self.newDataPacketCount - self.dispDataPacketCount - 1
                times = np.arange(self.dispDataPacketCount + self.packetCountLoop * 256,
                                  self.newDataPacketCount + self.packetCountLoop * 256, 1 / 19)
            else:
                jump = 255 - self.dispDataPacketCount + self.newDataPacketCount
                times = np.arange(self.dispDataPacketCount + self.packetCountLoop * 256,
                                  self.newDataPacketCount + (self.packetCountLoop + 1) * 256, 1 / 19)
                self.packetCountLoop += 1

            if jump:
                self.packetLostCount += jump
                print('The % of lost packet is', self.packetLostCount/(self.packetRecieved+self.packetLostCount))

            for i in range(0, 19*jump):
                self.q.put([times[i], 2020])
                self.audioVector[self.ptr] = 1743
                self.ptr += 1

            for i in range(0, 19):
                self.q.put([times[i+19*jump], self.unpackedData[i]])
                self.audioVector[self.ptr] = self.unpackedData[i]
                self.ptr += 1
