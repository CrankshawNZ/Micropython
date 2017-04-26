from machine import Pin, I2C

class MCP23008(object):
    """A convenience class to interact with the MCP23008.
    Based on the datasheet found at http://ww1.microchip.com/downloads/en/DeviceDoc/21919e.pdf
    Author: joe@crankshaw.org """

    #Some defaults and init
    def __init__(self, i2c = None, address = 32 ):
        """Initialise the MCP23008 object, defaults to address 32 / "\\x20" and
        scl pin 4, sda pin 5 if no i2c object passed."""

        # Register values
        self._registers = {}
        self._registers['IODIR'] = 0   # b'\x00'
        self._registers['IPOL'] = 1    # b'\x01'
        self._registers['GPINTEN'] = 2 # b'\x02'
        self._registers['DEFVAL'] = 3  # b'\x03'
        self._registers['INTCON'] = 4  # b'\x04'
        self._registers['IOCON'] = 5   # b'\x05'
        self._registers['GPPU'] = 6    # b'\x06'
        self._registers['INTF'] = 7    # b'\x07'
        self._registers['INTCAP'] = 8  # b'\x08'
        self._registers['GPIO'] = 9    # b'\x09'
        self._registers['OLAT'] = 10   # b'\x0A'

        # Premake some buffers for interrupt safeness, only the likely ones for now
        self._intcap_buf = bytearray(1)
        self._intf_buf = bytearray(1)
        self._gpio_buf = bytearray(1)

        # I2C Address
        self._address = address        # b'\x20' = 32.  so 32+ for addresses

        # I2C Object
        if i2c == None:                # A default that is useful during dev
            self._i2c = I2C(scl=Pin(4), sda=Pin(5), freq=100000)
        else:
            self._i2c = i2c



    ########
    # Access to the chips various registers.
    ########

    @property
    def IODIR(self):
        """Controls the direction of the data I/O.
        When a bit is set, the corresponding pin becomes an
        input. When a bit is clear, the corresponding pin
        becomes an output"""

        #print ("IODIR getter called")
        # read i2c
        return self._i2c.readfrom_mem(self._address, self._registers['IODIR'], 1 )

    @IODIR.setter
    def IODIR(self, value):
        #print ("IODIR setter called with value: %s" % value)
        
        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['IODIR'], value)



    @property
    def IPOL(self):
        """The IPOL register allows the user to configure the
        polarity on the corresponding GPIO port bits.
        If a bit is set, the corresponding GPIO register bit will
        reflect the inverted value on the pin. """

        #print ("IPOL getter called")
        # read i2c
        return self._i2c.readfrom_mem(self._address, self._registers['IPOL'], 1 )

    @IPOL.setter
    def IPOL(self, value):

        #print ("IPOL setter called")
        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['IPOL'], value)



    @property
    def GPINTEN(self):
        """The GPINTEN register controls the interrupt-onchange feature for each pin.
        If a bit is set, the corresponding pin is enabled for
        interrupt-on-change. The DEFVAL and INTCON
        registers must also be configured if any pins are
        enabled for interrupt-on-change."""

        #print ("GPINTEN getter called")
        # read i2c
        return self._i2c.readfrom_mem(self._address, self._registers['GPINTEN'], 1 )

    @GPINTEN.setter
    def GPINTEN(self, value):
        #print ("GPINTEN setter called")
        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['GPINTEN'], value)



    @property
    def DEFVAL(self):
        """The default comparison value is configured in the
        DEFVAL register. If enabled (via GPINTEN and
        INTCON) to compare against the DEFVAL register, an
        opposite value on the associated pin will cause an
        interrupt to occur"""

        #print ("DEFVAL getter called")
        # read from i2c
        return self._i2c.readfrom_mem(self._address, self._registers['DEFVAL'], 1 )

    @DEFVAL.setter
    def DEFVAL(self, value):

        #print ("DEFVAL setter called")
        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['DEFVAL'], value)



    @property
    def INTCON(self):
        """The INTCON register controls how the associated pin
        value is compared for the interrupt-on-change feature.
        If a bit is set, the corresponding I/O pin is compared
        against the associated bit in the DEFVAL register. If a
        bit value is clear, the corresponding I/O pin is compared
        against the previous value."""

        #print ("INTCON getter called")
        # read from i2c
        return self._i2c.readfrom_mem(self._address, self._registers['INTCON'], 1 )

    @INTCON.setter
    def INTCON(self, value):

        #print ("INTCON setter called")
        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['INTCON'], value)



    @property
    def IOCON(self):
        """The IOCON register contains several bits for
        configuring the device:

        * The Sequential Operation (SEQOP) controls the
        incrementing function of the address pointer. If the
        address pointer is disabled, the address pointer
        does not automatically increment after each byte
        is clocked during a serial transfer. This feature is
        useful when it is desired to continuously poll
        (read) or modify (write) a register.
        Bit 5  SEQOP: Sequential Operation mode bit.
        1 = Sequential operation disabled, address pointer does not increment.
        0 = Sequential operation enabled, address pointer increments.
        
        * The Slew Rate (DISSLW) bit controls the slew
        rate function on the SDA pin. If enabled, the SDA
        slew rate will be controlled when driving from a
        high to a low.
        Bit 4  DISSLW: Slew Rate control bit for SDA output.
        1 = Slew rate disabled.
        0 = Slew rate enabled.
        
        * The Hardware Address Enable (HAEN) control bit
        enables/disables the hardware address pins (A1,
        A0) on the MCP23S08. This bit is not used on the
        MCP23008. The address pins are always enabled
        on the MCP23008.
        
        * The Open-Drain (ODR) control bit enables/
        disables the INT pin for open-drain configuration.
        Bit 2  ODR: This bit configures the INT pin as an open-drain output.
        1 = Open-drain output (overrides the INTPOL bit).
        0 = Active driver output (INTPOL bit sets the polarity).
        
        * The Interrupt Polarity (INTPOL) control bit sets
        the polarity of the INT pin. This bit is functional
        only when the ODR bit is cleared, configuring the
        INT pin as active push-pull.
        Bit 1  INTPOL: This bit sets the polarity of the INT output pin.
        1 = Active-high.
        0 = Active-low. """

        #print ("IOCON getter called")
        # read from i2c
        return self._i2c.readfrom_mem(self._address, self._registers['IOCON'], 1 )

    @IOCON.setter
    def IOCON(self, value):

        #print ("IOCON setter called")
        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['IOCON'], value)



    @property
    def GPPU(self):
        """The GPPU register controls the pull-up resistors for the
        port pins. If a bit is set and the corresponding pin is
        configured as an input, the corresponding port pin is
        internally pulled up with a 100 kohm resistor."""

        # print ("GPPU getter called")
        # read from i2c
        return self._i2c.readfrom_mem(self._address, self._registers['GPPU'], 1 )

    @GPPU.setter
    def GPPU(self, value):

        # print ("GPPU setter called")
        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['GPPU'], value)



    @property
    def INTF(self):
        """The INTF register reflects the interrupt condition on the
        port pins of any pin that is enabled for interrupts via the
        GPINTEN register. A 'set' bit indicates that the associated
        pin caused the interrupt.

        This register is 'read-only'. Writes to this register will be
        ignored."""

        #print ("INTF getter called")
        # read from i2c - Use an existing buf to be interrupt safe

        self._i2c.readfrom_mem_into(self._address, self._registers['INTF'], self._intf_buf )
        return self._intf_buf



    @property
    def INTCAP(self):
        """The INTCAP register captures the GPIO port value at
        the time the interrupt occurred. The register is 'readonly'
        and is updated only when an interrupt occurs. The
        register will remain unchanged until the interrupt is
        cleared via a read of INTCAP or GPIO."""

        #print ("INTCAP getter called")
        # read via i2c - Use an existing buf to be interrupt safe

        self._i2c.readfrom_mem_into(self._address, self._registers['INTCAP'], self._intcap_buf )
        return self._intcap_buf


    @property
    def GPIO(self):
        """The GPIO register reflects the value on the port.
        Reading from this register reads the port. Writing to this
        register modifies the Output Latch (OLAT) register."""
        # print ("GPIO getter called")
        self._i2c.readfrom_mem_into(self._address, self._registers['GPIO'], self._gpio_buf )
        return self._gpio_buf

    @GPIO.setter
    def GPIO(self, value):
        # print ("GPIO setter called")
        self._i2c.writeto_mem(self._address, self._registers['GPIO'], value)


    @property
    def OLAT(self):
        """The OLAT register provides access to the output
        latches. A read from this register results in a read of the
        OLAT and not the port itself. A write to this register
        modifies the output latches that modify the pins
        configured as outputs. """
        #print ("OLAT getter called")
        return self._i2c.readfrom_mem(self._address, self._registers['OLAT'], 1 )

    @OLAT.setter
    def OLAT(self, value):
        #print ("OLAT setter called")
        self._i2c.writeto_mem(self._address, self._registers['OLAT'], value)


    #########
    ## Helper Methods
    #########

    def _setBitHigh(self, buf, bit_num):

        # Build a single bit at right pin.
        bit = 1 << bit_num

        # Or against in buf to ensure that bit is high
        out_buf = buf[0] | bit
        
        return bytes([out_buf])

    def _setBitLow(self, buf, bit_num):

        # Build a zero at right position
        bit = 1 << bit_num

        # AND against the inverted bit
        out_buf = buf[0] & ~(bit)

        return bytes([out_buf])


    ########
    ## Convenience Methods to deal with individual pins.
    #######

    def setPinDir(self, pin, direction):
        """Set a pins direction, 0 for output, 1 for input.
           Pin: 0 - 7
           Direction: [ 0, 1 ]"""

        # Get current state.
        current_state = self.IODIR

        # Flip the corresponding bit for the pin (yay for 1 to 1 pin/bit mapping)
        if direction == 0:
            self.IODIR = self._setBitLow(current_state, pin)
        elif direction == 1:
            self.IODIR = self._setBitHigh(current_state, pin)


    # Pullup resistors
    def setPullupOn(self,pin):
        """Turn on the pullup resistor for pin """
        current_state = self.GPPU
        self.GPPU = self._setBitHigh(current_state, pin)

    def setPullupOff(self,pin):
        """Turn off the pullup resistor for pin """
        current_state = self.GPPU
        self.GPPU = self._setBitHigh(current_state, pin)

    # Setting default values for interrupt comparisons.
    def setDefaultLow(self, pin):
        """Set default value for pin to low. Used to compare against for
        interupt generation"""
        current_state = self.DEFVAL
        self.DEFVAL = self._setBitLow(current_state, pin)

    def setDefaultHigh(self, pin):
        """Set default value for pin to high. Used to compare against for
        interupt generation"""
        current_state = self.DEFVAL
        self.DEFVAL = self._setBitHigh(current_state, pin)

    # Setting output pins high/low
    def setPinLow(self, pin):
        """Set a pin to low """
        # Get current state.
        current_state = self.OLAT
        # Set bit for pin.
        self.OLAT = self._setBitLow(current_state, pin)

    def setPinHigh(self, pin):
        """Set a pin to high """
        # Get current state.
        current_state = self.OLAT
        # Set bit for pin and apply to register
        self.OLAT = self._setBitHigh(current_state, pin)


    def readPin(self, pin):
        """Return low(0)/high(1) state for pin"""

        # Check if bit was up or down by AND'ing the isolated bit.
        # Crammed in like this to avoid allocating me (so it's interrupt safe)
        if (self.GPIO[0] & (1 << pin)) == 0:
            return 0
        else:
            return 1



