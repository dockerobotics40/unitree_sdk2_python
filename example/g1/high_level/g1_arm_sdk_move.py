import time
import sys
import numpy as np

# Importación de módulos de la SDK de Unitree para comunicación y control
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

# Constantes matemáticas
kPi = 3.141592654
mVo_w = 0.5236  # 30°
mVo_E = 0.7854  # 45°

class G1JointIndex:
    """
    Índices de las articulaciones del robot G1 de Unitree.
    Se incluyen piernas, brazos y cintura.
    """
    # Pierna izquierda
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Pierna derecha
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    # Cintura
    WaistYaw = 12
    WaistRoll = 13        # No válido para G1 23DoF/29DoF con cintura bloqueada
    WaistA = 13           # No válido para G1 23DoF/29DoF con cintura bloqueada
    WaistPitch = 14       # No válido para G1 23DoF/29DoF con cintura bloqueada
    WaistB = 14           # No válido para G1 23DoF/29DoF con cintura bloqueada

    # Brazo izquierdo
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # No válido para G1 23DoF
    LeftWristYaw = 21     # No válido para G1 23DoF

    # Brazo derecho
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # No válido para G1 23DoF
    RightWristYaw = 28    # No válido para G1 23DoF

    kNotUsedJoint = 29  # Articulación no utilizada (peso)

class Custom:
    """Clase para controlar los movimientos del robot G1 de Unitree."""
    def __init__(self):
        self.control_dt_ = 0.02  # Intervalo de control (20 ms)
        self.kp = 60.  # Ganancia proporcional
        self.kd = 1.5  # Ganancia derivativa
        self.weight = 0.
        self.weight_rate = 0.2
        self.dq = 0.
        self.tau_ff = 0.
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False 
        self.current_stage = 0  # Control de etapas

        # Lista de articulaciones controladas
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
            G1JointIndex.WaistYaw,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistPitch
        ]

        # Configuración de posiciones objetivo por el usuario
        self.target_pos = self.get_user_joint_positions()

    def get_user_joint_positions(self):
        """Permite al usuario configurar las posiciones deseadas de las articulaciones."""
        target_pos = {joint: 0.0 for joint in self.arm_joints}  # Inicializar en 0
        print("\nConfiguración de posiciones articulares:")
        for joint in self.arm_joints:
            joint_name = [name for name, value in G1JointIndex.__dict__.items() if value == joint][0]
            try:
                pos = float(input(f"Ingrese posición para {joint_name} (rad) o deje vacío para 0: ") or 0)
                target_pos[joint] = pos
            except ValueError:
                print(f"Entrada no válida para {joint_name}. Se establecerá en 0.")

        return target_pos

    def Init(self):
        """Inicializa la comunicación con el robot."""
        # Publicador para enviar comandos al robot
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # Suscriptor para recibir el estado del robot
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        """Inicia el control del robot en un bucle de espera por entrada del usuario."""
        while self.first_update_low_state == False:
            time.sleep(1)
        self.run_sequence()

    def LowStateHandler(self, msg: LowState_):
        """Maneja la recepción de los estados del robot."""
        self.low_state = msg
        if self.first_update_low_state == False:
            self.first_update_low_state = True

    def run_sequence(self):
        """Ejecuta la secuencia de movimientos basados en la entrada del usuario."""
        stages = [
            ("Mover el brazo a la posición cero", self.move_to_zero),
            ("Abrir los brazos según las posiciones objetivo", self.move_to_target),
            ("Volver a la posición cero", self.move_to_zero),
            ("Liberar el control", self.release_control)
        ]

        for stage_desc, stage_func in stages:
            input(f"\n{stage_desc}. Presione Enter para continuar...")
            stage_func()

        print("\nSecuencia completa. Saliendo...")
        sys.exit(0)

    def move_to_zero(self):
        """Lleva todas las articulaciones a la posición cero."""
        self.execute_movement({joint: 0.0 for joint in self.arm_joints})

    def move_to_target(self):
        """Mueve las articulaciones a la posición objetivo definida por el usuario."""
        self.execute_movement(self.target_pos)

    def release_control(self):
        """Libera el control de manera progresiva."""
        duration = 2.0  # Duración de la transición en segundos
        steps = int(duration / self.control_dt_)  # Cantidad de pasos según dt

        for step in range(steps):
            ratio = (step + 1) / steps  # Interpolación lineal de 0 a 1
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1 - ratio  # De 1 (activo) a 0 (desactivado)

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)  # Actualizar CRC
            self.arm_sdk_publisher.Write(self.low_cmd)  # Enviar comando

            time.sleep(self.control_dt_)  # Esperar 20ms por cada iteración

        print("\nControl liberado completamente.")

    def execute_movement(self, target_positions):
        """Ejecuta un movimiento suave hacia las posiciones deseadas."""
        duration = 5.0  # Duración de la transición en segundos
        steps = int(duration / self.control_dt_)

        for step in range(steps):
            ratio = (step + 1) / steps
            for joint in self.arm_joints:
                current_pos = self.low_state.motor_state[joint].q
                target_pos = target_positions[joint]
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1 - ratio) * current_pos + ratio * target_pos
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            time.sleep(self.control_dt_)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Uso: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    print("ADVERTENCIA: Asegúrese de que no haya obstáculos cerca del robot.")
    input("Presione Enter para continuar...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()
