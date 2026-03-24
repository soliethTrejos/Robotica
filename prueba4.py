from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

motor_izq = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motor_der = Motor(Port.F, Direction.CLOCKWISE)
#Sensor de abajo E, sensor de garra A
sensor_linea = ColorSensor(Port.E)
garra_trasera = Motor(Port.C) 

robot = DriveBase(motor_izq, motor_der, wheel_diameter=56, axle_track=120)

#Gyro sensor activo
robot.use_gyro(True)

robot.settings(
    straight_speed=960,
    straight_acceleration=960,
    turn_rate=220,
    turn_acceleration=960
)

def cm(valor):
    return valor * 10

wait(300)

robot.straight(cm(-2.5))
robot.turn(-97)
robot.straight(cm(2.5))

#Gyro desactivado antes del seguimiento de linea
#robot.use_gyro(False)

# Variables para la corrección
velocidad = 300
umbral = 35
kp = 0.8
kd = 1.5
error_anterior = 1

while True:
    valor = sensor_linea.reflection()
    error = valor - umbral
    derivada = error - error_anterior

    correccion = (kp * error) + (kd * derivada)

    # Limita la corrección para que no gire demasiado brusco
    if correccion > 80:
        correccion = 80
    if correccion < -80:
        correccion = -80

    robot.drive(velocidad, correccion)

    # Si detecta negro fuerte, se detiene y retrocede
    if valor <= 20:
        robot.stop()      
        break
    error_anterior = error
    wait(10)

#Gyro activado de nuevo
#robot.use_gyro(True)
wait(300)

robot.straight(cm(95))
robot.turn(-125) 
robot.straight(cm(-3))
garra_trasera.run_angle(-300, -90) 
robot.turn(100)

