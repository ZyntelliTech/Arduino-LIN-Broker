Put BOTH files in the same Arduino sketch folder:

arduino_lin_broker/
  arduino_lin_broker.ino
  build_opt.h

Blue Pill LIN wiring:
  Master-side transceiver:
    RX  -> PA10
    TX  -> PA9
    SLP -> PA8

  Slave-side transceiver:
    RX  -> PA3
    TX  -> PA2
    SLP -> PA5

The build_opt.h file enables Serial2 in STM32duino and forces it to PA3/PA2.
Do not delete build_opt.h.
