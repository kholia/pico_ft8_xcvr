# Reciprocal pulse counter from 'horuable' (RecipCounter1.py)
# https://www.raspberrypi.org/forums/viewtopic.php?f=146&t=306250&p=1832034#p1832034
# https://github.com/jbeale1/pico/blob/main/RecipCounter1.py - A few mods by J.Beale 24-March-2021

import rp2
import math
import utime
from rp2 import PIO, asm_pio
from machine import UART, Pin
from micropython import const

# machine CPU frequency defaults to 125 MHz, but can be set up to 250 MHz (or more)
# fractional calibration changes as well

#fcal = 1.0000150  # (at 125 MHz)
#fcal = 1.0000073  # (at 250 MHz)
fcal = 1

#MFREQ = 125000000
MFREQ = 250000000
MCAL  = int(MFREQ * fcal)       # calibrated value for this board at given CPU freq

global gateVal
# gateVal = int(MFREQ/10000)
gateVal = int(MFREQ)


@asm_pio(sideset_init=PIO.OUT_HIGH)
def gate():
    """PIO to generate gate signal."""
    mov(x, osr)                                            # load gate time (in clock pulses) from osr
    wait(0, pin, 0)                                        # wait for input to go low
    wait(1, pin, 0)                                        # wait for input to go high - effectively giving us rising edge detection
    label("loopstart")
    jmp(x_dec, "loopstart") .side(0)                       # keep gate low for time programmed by setting x reg
    wait(0, pin, 0)                                        # wait for input to go low
    wait(1, pin, 0) .side(1)                               # set gate to high on rising edge
    irq(block, 0)                                          # set interrupt 0 flag and wait for system handler to service interrupt
    wait(1, irq, 4)                                        # wait for irq from clock counting state machine
    wait(1, irq, 5)                                        # wait for irq from pulse counting state machine

@asm_pio()
def clock_count():
    """PIO for counting clock pulses during gate low."""
    mov(x, osr)                                            # load x scratch with max value (2^32-1)
    wait(1, pin, 0)                                        # detect falling edge
    wait(0, pin, 0)                                        # of gate signal
    label("counter")
    jmp(pin, "output")                                     # as long as gate is low //
    jmp(x_dec, "counter")                                  # decrement x reg (counting every other clock cycle - have to multiply output value by 2)
    label("output")
    mov(isr, x)                                            # move clock count value to isr
    push()                                                 # send data to FIFO
    irq(block, 4)                                          # set irq and wait for gate PIO to acknowledge

@asm_pio(sideset_init=PIO.OUT_HIGH)
def pulse_count():
    """PIO for counting incoming pulses during gate low."""
    mov(x, osr)                                            # load x scratch with max value (2^32-1)
    wait(1, pin, 0)
    wait(0, pin, 0) .side(0)                               # detect falling edge of gate
    label("counter")
    wait(0, pin, 1)                                        # wait for rising
    wait(1, pin, 1)                                        # edge of input signal
    jmp(pin, "output")                                     # as long as gate is low //
    jmp(x_dec, "counter")                                  # decrement x req counting incoming pulses (probably will count one pulse less than it should - to be checked later)
    label("output")
    mov(isr, x) .side(1)                                   # move pulse count value to isr and set pin to high to tell clock counting sm to stop counting
    push()                                                 # send data to FIFO
    irq(block, 5)                                          # set irq and wait for gate PIO to acknowledge


def init_sm(freq, input_pin, gate_pin, pulse_fin_pin):
    """Starts state machines."""
    global gateVal

    gate_pin.value(1)
    pulse_fin_pin.value(1)
    max_count = const((1 << 32) - 1)

    sm0 = rp2.StateMachine(0, gate, freq=freq, in_base=input_pin, sideset_base=gate_pin)
    sm0.put(gateVal)
    sm0.exec("pull()")

    sm1 = rp2.StateMachine(1, clock_count, freq=freq, in_base=gate_pin,
                           jmp_pin=pulse_fin_pin)
    sm1.put(max_count)
    sm1.exec("pull()")

    sm2 = rp2.StateMachine(2, pulse_count, freq=freq, in_base=gate_pin,
                            sideset_base = pulse_fin_pin, jmp_pin=gate_pin)
    sm2.put(max_count-1)
    sm2.exec("pull()")

    sm1.active(1)
    sm2.active(1)
    sm0.active(1)

    return sm0, sm1, sm2

# Calculate the crc (16-bit) for the message
def calcChecksum(packet):
    CK_A,CK_B = 0, 0
    for i in range(len(packet)):
        CK_A = CK_A + packet[i]
        CK_B = CK_B + CK_A

    # ensure unsigned byte range
    CK_A = CK_A & 0xFF
    CK_B = CK_B & 0xFF
    return CK_A, CK_B


# Send a message to set a new frequency and locked frequency Note: This does
# not wait for an acknowledge, nor poll the actual settings
def configureTimepulse(freq, freqLock, pulseRatio, pulseRatioLock):
  factor = math.pow(2.0, -32.0);
  pulseRatio = 1 - pulseRatio;
  dc = int(pulseRatio / factor);
  pulseRatioLock = 1 - pulseRatioLock;
  dcLock = int(pulseRatioLock / factor);

  message = [0] * 40
  message[0] =  0xB5 # header
  message[1] =  0x62  # header
  message[2] =  0x06  # class
  message[3] =  0x31  # id
  message[4] =  0x20  # length
  message[5] =  0x00  # length
  message[6] =  0x00  # time pulse selection
  message[7] =  0x01  # version
  message[8] =  0x00  # reserved
  message[9] =  0x00  # reserved
  message[10] = 0x32  # antenna cable delay (here fixed)
  message[11] = 0x00  # antenna cable delay
  message[12] = 0x00  # rf group delay (here fixed)
  message[13] = 0x00  # rf group delay
  message[14] = (freq >>  0) & 0xFF  # frequency
  message[15] = (freq >>  8) & 0xFF  # frequency
  message[16] = (freq >> 16) & 0xFF  # frequency
  message[17] = (freq >> 24) & 0xFF  # frequency
  message[18] = (freqLock >>  0) & 0xFF  # frequency on lock
  message[19] = (freqLock >>  8) & 0xFF  # frequency on lock
  message[20] = (freqLock >> 16) & 0xFF  # frequency on lock
  message[21] = (freqLock >> 24) & 0xFF  # frequency on lock
  message[22] = (dc >>  0) & 0xFF  # dutycycle
  message[23] = (dc >>  8) & 0xFF  # dutycycle
  message[24] = (dc >> 16) & 0xFF  # dutycycle
  message[25] = (dc >> 24) & 0xFF  # dutycycle
  message[26] = (dcLock >>  0) & 0xFF  # dutycycle on lock
  message[27] = (dcLock >>  8) & 0xFF  # dutycycle on lock
  message[28] = (dcLock >> 16) & 0xFF  # dutycycle on lock
  message[29] = (dcLock >> 24) & 0xFF  # dutycycle on lock
  message[30] = 0x00  # user configured delay
  message[31] = 0x00  # user configured delay
  message[32] = 0x00  # user configured delay
  message[33] = 0x00  # user configured delay
  message[34] = 0xEF  # flags
  message[35] = 0x00  # flags
  message[36] = 0x00  # flags
  message[37] = 0x00  # flags
  message[38], message[39] = calcChecksum(message[2:40-2]) #  crc will be included after calculation
  return message


if __name__ == "__main__":
    from machine import Pin, freq
    import uarray as array
    global gateVal

    # Configure the (>=) NEO-6M GPS module
    uart1 = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))
    message = configureTimepulse(1, 12000000, 0.5, 0.5)
    # message = configureTimepulse(5, 8000000, 0.5, 0.5)
    uart1.write(bytes(message))

    freq(MFREQ)      # set CPU frequency; not necessarily the default 125 MHz
    update_flag = False
    data = array.array("I", [0, 0])
    def counter_handler(sm):
        #print("IRQ")
        global update_flag
        if not update_flag:
            sm0.put(gateVal)
            sm0.exec("pull()")
            data[0] = sm1.get() # clock count
            data[1] = sm2.get() # pulse count
            update_flag = True

    sm0, sm1, sm2 = init_sm(MFREQ, Pin(15, Pin.IN, Pin.PULL_UP), Pin(14, Pin.OUT), Pin(13, Pin.OUT))
    #sm0, sm1, sm2 = init_sm(MFREQ, Pin(16, Pin.IN, Pin.PULL_UP), Pin(14, Pin.OUT), Pin(13, Pin.OUT))
    sm0.irq(counter_handler)

    print("n,msec,clocks,pulses,Hz") # CSV file column headers
    i = 0
    rCtr = 0
    while True:
        if update_flag:
            msec=utime.ticks_ms()
            rCtr += 1
            update_flag = False
            #if (rCtr % 100) == 0:
            if True:
              clock_count = (max_count - data[0]) # units of 2 * Tcpu  (Tcpu = 1/MFREQ)
              pulse_count = max_count - data[1]
              freq = pulse_count * ( MCAL / (2*clock_count))  # calibration constant
              print("{:d},{:d},{:d},{:d},{:0.5f}".format(i,msec,clock_count,pulse_count,freq))
              i += 1
