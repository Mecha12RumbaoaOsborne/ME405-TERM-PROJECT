from pyb import Pin, Timer

tim4 = Timer(4)
tim4.init(prescaler = 0, period =3)
PB6 = Pin(Pin.cpu.B6, mode = Pin.OUT_PP)
clk = tim4.channel(1, mode=Timer.PWM, pin = PB6, pulse_width = 2)

while True:
    try:
        print(PB6.value())
    except KeyboardInterrupt:
        break
