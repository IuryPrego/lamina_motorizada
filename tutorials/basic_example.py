from waveplate import WavePlate

with WavePlate("COM4", disable_when_idle=False) as plate:
    plate.configure(min_speed=20, max_speed=90, acceleration=150)  # opcional
    plate.home()                       # zero mecânico

    plate.move_to(45)                  # absoluto (espera)
    plate.move_by(-10)                 # relativo (espera)
    plate.get_position()               # onde está

    plate.move_to(180, wait=False)     # sem bloquear
    while plate.is_moving(): ...
    plate.wait_move()                  # ou stop()

    plate.get_settings()               # valores atuais
    plate.restore_defaults()
    plate.limit_switch_triggered()     # True/False
    plate.enable(); plate.disable()
    plate.disable_when_idle(True)
