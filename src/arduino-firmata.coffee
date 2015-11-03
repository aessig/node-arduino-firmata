'use strict'

events = require 'eventemitter2'
{SerialPort} = serialport = require 'serialport'

debug = require('debug')('arduino-firmata')

exports = module.exports = class ArduinoFirmata extends events.EventEmitter2

  @TRANSLATE = {
    16: { # __AVR_ATmega1280__ || __AVR_ATmega2560__ --> Arduino Mega
      A0: 54,
      A1: 55,
      A2: 56,
      A3: 57,
      A4: 58,
      A5: 59,
      A6: 60,
      A7: 61,
      A8: 62,
      A9: 63,
      A10: 64,
      A11: 65,
      A12: 66,
      A13: 67,
      A14: 68,
      A15: 69
    },
    32: { # __AVR_ATmega328P__ || __AVR_ATmega168__ --> Arduino Uno
      A0: 14,
      A1: 15,
      A2: 16,
      A3: 17,
      A4: 18,
      A5: 19
    }
    48: { # __AVR_ATmega32U4__ || __AVR_ATmega16U4__ --> Arduino Leonardo
      A0: 18,
      A1: 19,
      A2: 20,
      A3: 21,
      A4: 22,
      A5: 23
    }
    64: { # undefind ---> Arduino Uno
      A0: 14,
      A1: 15,
      A2: 16,
      A3: 17,
      A4: 18,
      A5: 19
    }
  }

  @Status = {
    CLOSE: 0
    OPEN: 1
  }

  @INPUT  = 0
  @FIRMATA_INPUT_PULLUP = 0x10
  @OUTPUT = 1
  @ANALOG = 2
  @PWM    = 3
  @SERVO  = 4
  @SHIFT  = 5
  @I2C    = 6
  @LOW    = 0
  @HIGH   = 1

  @MAX_DATA_BYTES  = 32
  @DIGITAL_MESSAGE = 0x90 # send data for a digital port
  @ANALOG_MESSAGE  = 0xE0 # send data for an analog pin (or PWM)
  @REPORT_ANALOG   = 0xC0 # enable analog input by pin
  @REPORT_DIGITAL  = 0xD0 # enable digital input by port
  @SET_PIN_MODE    = 0xF4 # set a pin to INPUT/OUTPUT/PWM/etc
  @REPORT_VERSION  = 0xF9 # report firmware version
  @REPORT_FIRMWARE = 0x79 # report firmware
  @SYSTEM_RESET    = 0xFF # reset from MIDI
  @START_SYSEX     = 0xF0 # start a MIDI SysEx message
  @END_SYSEX       = 0xF7 # end a MIDI SysEx message

  @list: (callback) ->
    serialport.list (err, ports) ->
      return callback err if err
      devices = []
      for port in ports
        devices.push port.comName if /usb|acm|com\d+/i.test port.comName
      callback null, devices

  constructor: ->
    @status = ArduinoFirmata.Status.CLOSE
    @wait_for_data = 0
    @execute_multi_byte_command = 0
    @multi_byte_channel = 0
    @stored_input_data = []
    @parsing_sysex = false
    @sysex_bytes_read = 0
    @digital_output_data = [0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0]
    @digital_input_data  = [0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0]
    @analog_input_data   = [0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0]
    @boardVersion = null
    @boardName = 0x40

  isOldArduinoDevice: ->
    return /usbserial|USB/.test @serialport_name

  trans: (pin)->
    if typeof pin is 'string' and pin.charAt(0) is 'A'
      return ArduinoFirmata.TRANSLATE[@boardName][pin]
    else
      return pin

  connect: (@serialport_name, opts={baudrate: 57600}) ->
    opts.parser = serialport.parsers.raw
    unless @serialport_name
      ArduinoFirmata.list (err, devices) =>
        @connect devices[0], opts
      return @

    @once 'boardReady', ->
      debug 'boardReady'
      io_init_wait = if @isOldArduinoDevice()
        debug "old arduino device found #{@serialport_name}"
        3000
      else
        debug "new arduino device found #{@serialport_name}"
        100
      debug "wait #{io_init_wait}(msec)"
      setTimeout =>
        for i in [0...6]
          @write [(ArduinoFirmata.REPORT_ANALOG | i), 1]
        for i in [0...2]
          @write [(ArduinoFirmata.REPORT_DIGITAL | i), 1]
        debug 'init IO ports'
        @emit 'connect'
      , io_init_wait

    @serialport = new SerialPort @serialport_name, opts
    @serialport.once 'open', =>
      cid = setInterval =>
        debug 'request REPORT_VERSION'
        @write [ArduinoFirmata.START_SYSEX, ArduinoFirmata.REPORT_FIRMWARE, ArduinoFirmata.END_SYSEX]
        @write [ArduinoFirmata.REPORT_VERSION]
      , 500
      @once 'boardVersion', (version) =>
        @once 'reportFirmware', (name) =>
          clearInterval cid
          @status = ArduinoFirmata.Status.OPEN
          @emit 'boardReady'
      @serialport.on 'data', (data) =>
        for byte in data
          @process_input byte

    return @

  isOpen: ->
    return @status is ArduinoFirmata.Status.OPEN

  close: (callback) ->
    @status = ArduinoFirmata.Status.CLOSE
    @serialport.close callback

  reset: (callback) ->
    @write [ArduinoFirmata.SYSTEM_RESET], callback

  write: (bytes, callback) ->
    @serialport.write bytes, callback

  sysex: (command, data=[], callback) ->
    ## http://firmata.org/wiki/V2.1ProtocolDetails#Sysex_Message_Format
    data = data.map (i) ->
      return i & 0b1111111  # 7bit
    write_data =
      [ArduinoFirmata.START_SYSEX, command].
        concat data, [ArduinoFirmata.END_SYSEX]
    @write write_data, callback

  pinMode: (pin, mode, callback) ->
    pin = @trans pin # Pin translation
    switch mode
      when true
        mode = ArduinoFirmata.OUTPUT
      when false
        mode = ArduinoFirmata.INPUT
    @write [ArduinoFirmata.SET_PIN_MODE, pin, mode], callback

  digitalWrite: (pin, value, callback) ->
    # @pinMode pin, ArduinoFirmata.OUTPUT
    port_num = (pin >>> 3) & 0x0F
    if value is 0 or value is false
      @digital_output_data[port_num] &= ~(1 << (pin & 0x07))
    else
      @digital_output_data[port_num] |= (1 << (pin & 0x07))
    @write [ (ArduinoFirmata.DIGITAL_MESSAGE | port_num),
             (@digital_output_data[port_num] & 0x7F),
             (@digital_output_data[port_num] >>> 7) ],
           callback

  analogWrite: (pin, value, callback) ->
    value = Math.floor value
    # @pinMode pin, ArduinoFirmata.PWM
    @write [ (ArduinoFirmata.ANALOG_MESSAGE | (pin & 0x0F)),
             (value & 0x7F),
             (value >>> 7) ],
           callback

  servoWrite: (pin, angle, callback) ->
    # @pinMode pin, ArduinoFirmata.SERVO
    @write [ (ArduinoFirmata.ANALOG_MESSAGE | (pin & 0x0F)),
             (angle & 0x7F),
             (angle >>> 7) ],
           callback

  digitalRead: (pin) ->
    return ((@digital_input_data[pin >>> 3] >>> (pin & 0x07)) & 0x01) > 0

  analogRead: (pin) ->
    return @analog_input_data[pin]

  queryFirmware: (callback) ->
    @write [ArduinoFirmata.START_SYSEX, ArduinoFirmata.REPORT_FIRMWARE, ArduinoFirmata.END_SYSEX], callback

  process_input: (input_data) ->
    if @parsing_sysex
      if input_data is ArduinoFirmata.END_SYSEX
        @parsing_sysex = false
        sysex_command = @stored_input_data[0]
        sysex_data = @stored_input_data[1...@sysex_bytes_read]
        if sysex_command is ArduinoFirmata.REPORT_FIRMWARE
          @boardName = sysex_data[1]
          @emit 'reportFirmware', @boardName
        else
          @emit 'sysex', {command: sysex_command, data: sysex_data}
      else
        @stored_input_data[@sysex_bytes_read] = input_data
        @sysex_bytes_read += 1
    else if @wait_for_data > 0 and input_data < 128
      @wait_for_data -= 1
      @stored_input_data[@wait_for_data] = input_data
      if @execute_multi_byte_command isnt 0 and @wait_for_data is 0
        switch @execute_multi_byte_command
          when ArduinoFirmata.DIGITAL_MESSAGE
            input_data = (@stored_input_data[0] << 7) + @stored_input_data[1]
            diff = @digital_input_data[@multi_byte_channel] ^ input_data
            @digital_input_data[@multi_byte_channel] = input_data
            if @listeners('digitalChange').length > 0
              for i in [0..13]
                if ((0x01 << i) & diff) > 0
                  stat = (input_data&diff) > 0
                  @emit 'digitalChange',
                  {pin: i+@multi_byte_channel*8, value: stat, old_value: !stat}
          when ArduinoFirmata.ANALOG_MESSAGE
            analog_value = (@stored_input_data[0] << 7) + @stored_input_data[1]
            old_analog_value = @analogRead(@multi_byte_channel)
            @analog_input_data[@multi_byte_channel] = analog_value
            if old_analog_value != analog_value
              @emit 'analogChange', {
                pin: @multi_byte_channel,
                value: analog_value,
                old_value: old_analog_value
              }
          when ArduinoFirmata.REPORT_VERSION
            @boardVersion = "#{@stored_input_data[1]}.#{@stored_input_data[0]}"
            @emit 'boardVersion', @boardVersion
    else
      if input_data < 0xF0
        command = input_data & 0xF0
        @multi_byte_channel = input_data & 0x0F
      else
        command = input_data
      if command is ArduinoFirmata.START_SYSEX
        @parsing_sysex = true
        @sysex_bytes_read = 0
      else if command is ArduinoFirmata.DIGITAL_MESSAGE or
              command is ArduinoFirmata.ANALOG_MESSAGE or
              command is ArduinoFirmata.REPORT_VERSION
        @wait_for_data = 2
        @execute_multi_byte_command = command
