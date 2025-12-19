#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped

DISTANCIA_DESEJADA = 0.15   # 15 cm
Kp = 0.5                    # ganho mais suave
V_MAX = 0.2                 # limite de velocidade
SENSOR_MAX = 1.1            # acima disto consideramos "sem parede"
SENSOR_MIN = 0.05           # abaixo disto consideramos "encostado"
DEADBAND = 0.01             # 1 cm de zona morta

pub = None

def saturar(v):
    return max(-V_MAX, min(V_MAX, v))

def tof_callback(msg):
    global pub
    distancia = msg.range

    # Caso 1: demasiado longe / sem parede
    if distancia >= SENSOR_MAX:
        v = 0.0
        rospy.loginfo("Longe demais (dist=%.3f m) -> v=0.0", distancia)

    # Caso 2: demasiado perto (encostado)
    elif distancia <= SENSOR_MIN:
        v = 0.0
        rospy.loginfo("Demasiado perto/encostado (dist=%.3f m) -> v=0.0", distancia)

    else:
        # erro = distancia - desejada  (positivo: longe demais, negativo: perto demais)
        erro = distancia - DISTANCIA_DESEJADA

        # zona morta à volta dos 15 cm
        if abs(erro) < DEADBAND:
            v = 0.0
        else:
            v = saturar(Kp * erro)

        rospy.loginfo("dist=%.3f m  erro=%.3f m  v=%.3f", distancia, erro, v)

    # enviar comando para rodas
    cmd = WheelsCmdStamped()
    cmd.vel_left = v
    cmd.vel_right = v
    pub.publish(cmd)

def main():
    global pub
    rospy.init_node("p_controller_demo")

    # Robot name configurável (default: duckiegogo)
    robot = rospy.get_param("~robot", "duckie5")

    wheels_topic = f"/{robot}/wheels_driver_node/wheels_cmd"
    tof_topic    = f"/{robot}/front_center_tof_driver_node/range"

    pub = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
    rospy.Subscriber(tof_topic, Range, tof_callback)

    rospy.loginfo("P-Controller ativo. Sub: %s | Pub: %s", tof_topic, wheels_topic)
    rospy.spin()

if __name__ == "__main__":
    main()