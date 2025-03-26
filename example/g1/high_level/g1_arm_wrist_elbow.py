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
kPi_2 = 1.57079632  # π/2
mVo_w = 0.5236  # 30°
mVo_E = 0.7854 # 45°

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
    """
    Clase para controlar los movimientos del robot G1 de articulaciones superiores.
    Se encarga de inicializar la comunicación y definir las fases del movimiento.
    """
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02  # Intervalo de control (20 ms)
        self.duration_ = 3.0  # Duración de cada fase en segundos
        self.counter_ = 0
        self.weight = 0.
        self.weight_rate = 0.2
        self.kp = 60.  # Ganancia proporcional
        self.kd = 1.5  # Ganancia derivativa
        self.dq = 0.
        self.tau_ff = 0.
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  # Comando de bajo nivel
        self.low_state = None  # Estado del robot
        self.first_update_low_state = False  # Indica si se ha recibido el primer estado
        self.crc = CRC()
        self.done = False  # Indica si la secuencia de movimientos ha terminado

        # Posiciones objetivo para las articulaciones del brazo y la cintura
        self.target_pos = [
            0., 0.,  0., 0., mVo_w, 0., 0.,
            0., 0., 0., mVo_E, 0., 0., 0., 
            0, 0, 0
        ]

        # Índices de las articulaciones del brazo y la cintura que se controlarán
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

    def Init(self):
        """
        Inicializa la comunicación con el robot.
        """
        # Publicador para enviar comandos al robot
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # Suscriptor para recibir el estado del robot
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        """
        Inicia el control del robot en un hilo recurrente.
        """
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.first_update_low_state == False:
            time.sleep(1)

        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        """
        Maneja la recepción de los estados del robot.
        """
        self.low_state = msg

        if self.first_update_low_state == False:
            self.first_update_low_state = True

    def LowCmdWrite(self):
        """
        Define la secuencia de movimientos del robot.
        """
        self.time_ += self.control_dt_

        if self.time_ < self.duration_:
            # Etapa 1: Llevar el robot a la postura cero
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1  # 1: Activar arm_sdk, 0: Desactivar arm_sdk
            for i, joint in enumerate(self.arm_joints):
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.low_cmd.motor_cmd[joint].tau = 0. 
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q 
                self.low_cmd.motor_cmd[joint].dq = 0. 
                self.low_cmd.motor_cmd[joint].kp = self.kp 
                self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 3:
            # Etapa 2: Movimiento de muñeca y codo
            for i,joint in enumerate(self.arm_joints):
              ratio = np.clip((self.time_ - self.duration_) / (self.duration_ * 2), 0.0, 1.0)
              self.low_cmd.motor_cmd[joint].tau = 0. 
              self.low_cmd.motor_cmd[joint].q = ratio * self.target_pos[i] + (1.0 - ratio) * self.low_state.motor_state[joint].q 
              self.low_cmd.motor_cmd[joint].dq = 0. 
              self.low_cmd.motor_cmd[joint].kp = self.kp 
              self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 6:
            # Etapa 3: Volver a la postura cero
            for i,joint in enumerate(self.arm_joints):
              ratio = np.clip((self.time_ - self.duration_*3) / (self.duration_ * 3), 0.0, 1.0)
              self.low_cmd.motor_cmd[joint].tau = 0. 
              self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q
              self.low_cmd.motor_cmd[joint].dq = 0. 
              self.low_cmd.motor_cmd[joint].kp = self.kp 
              self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 7:
            # Etapa 4: Liberar arm_sdk
            for i,joint in enumerate(self.arm_joints):
              ratio = np.clip((self.time_ - self.duration_*6) / (self.duration_), 0.0, 1.0)
              self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  (1 - ratio) # 1: Activar arm_sdk, 0: Desactivar arm_sdk

        else:
            self.done = True  # Fin del movimiento

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

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

    while True:        
        time.sleep(1)
        if custom.done: 
           print("Done!")
           sys.exit(-1) 