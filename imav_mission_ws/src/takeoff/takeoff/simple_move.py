import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    DistanceSensor,
)


class PX4FlowPrecision(Node):
    def __init__(self):
        super().__init__('px4_flow_precision')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_pos_cb, sub_qos)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_cb, sub_qos)
        self.flow_sub = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self.flow_cb, sub_qos)

        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz
        self.counter = 0

        # ── Estado estimado del vehículo ─────────────────────────────────────
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_z   = 0.0
        self.current_yaw = 0.0

        # xy_valid: el EKF2 confirma que la posición XY es confiable.
        # Este flag es TRUE solo cuando el H-Flow está fusionando correctamente
        # (EKF2_OF_CTRL=1, señal de calidad OK, altura dentro de SENS_FLOW_MINHGT/MAXHGT).
        # Si está en FALSE, no comandamos movimiento horizontal.
        self.xy_valid = False

        # Altura medida por el sensor de distancia integrado del H-Flow.
        # Se usa como fuente de altitud (no current_z que viene del barométrico).
        self.current_distance = 0.0
        self.flow_quality     = 0

        # ── Posición bloqueada al armar ──────────────────────────────────────
        # Se congela mientras el dron está en tierra para evitar que el drift
        # inicial del EKF2 nos dé un setpoint erróneo.
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        # ── Objetivo del estado FORWARD ──────────────────────────────────────
        # Se calcula una sola vez al entrar en FORWARD, en coordenadas NED.
        # forward = pos_actual + 2 m × [cos(yaw), sin(yaw)]
        # Esto mueve el dron en la dirección en que está mirando.
        self.forward_target_x  = None
        self.forward_target_y  = None
        self.forward_distance  = 2.0  # metros

        # ── Parámetros de vuelo ──────────────────────────────────────────────
        self.target_altitude = 1.2   # Altura objetivo en metros (sobre el suelo)
        self.target_z        = -1.2  # Mismo valor en NED (Z negativo = arriba)
        self.hold_duration   = 3.0   # segundos de hover en HOLD y HOLD_FWD

        # ── Control de estados ───────────────────────────────────────────────
        self.state               = "INIT"
        self.hold_start_time     = None
        self.hold_fwd_start_time = None
        self.stable_ticks        = 0
        self.stable_ticks_needed = 10  # 1 segundo a 10 Hz

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        # xy_valid es el indicador clave del EKF2 para saber si el H-Flow
        # está siendo fusionado correctamente. Si está en False, la posición
        # XY reportada NO es confiable y no debemos comandar movimiento.
        self.xy_valid = msg.xy_valid

    def attitude_cb(self, msg: VehicleAttitude):
        # Extraer yaw (ángulo de guiñada) del cuaternión de actitud.
        # El cuaternión q = [w, x, y, z] en px4_msgs.
        q = msg.q
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg: DistanceSensor):
        self.current_distance = msg.current_distance
        self.flow_quality     = msg.signal_quality
        if self.counter % 10 == 0:
            self.get_logger().info(
                f"[SENSOR] calidad={self.flow_quality:3d} | "
                f"dist={self.current_distance:.3f} m | "
                f"xy_valid={self.xy_valid}"
            )

    # =========================================================================
    # LOOP PRINCIPAL (10 Hz)
    # =========================================================================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # ── Offboard heartbeat ────────────────────────────────────────────────
        # Debe publicarse a mínimo 2 Hz para que PX4 no salga del modo Offboard.
        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # ── Setpoint base (todo NaN = no se sobreescribe lo que no se asigne) ─
        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed     = float('nan')

        # ── Congelar XY mientras el dron está en tierra ───────────────────────
        # En tierra, el EKF2 empieza a estabilizarse y la posición puede
        # derivar unos centímetros. Bloqueamos los valores para no comandar
        # movimiento involuntario al entrar en Offboard.
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x   = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y   = self.locked_y   if self.locked_y   is not None else 0.0
        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0
        setpoint.yaw = safe_yaw

        # =========================================================================
        # MÁQUINA DE ESTADOS
        # =========================================================================

        if self.state == "INIT":
            # Esperar 2 s (20 ticks a 10 Hz) para que los topics del FMU
            # estén disponibles antes de enviar comandos.
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)  # MAV_CMD: modo OFFBOARD
                self.state = "ARMING"
                self.get_logger().info("INIT→ARMING: modo Offboard solicitado")

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)  # MAV_CMD_COMPONENT_ARM_DISARM
                self.get_logger().info(
                    f"ARMING→TAKEOFF: motores armados, "
                    f"subiendo a {self.target_altitude} m"
                )
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            # Mantener la posición XY bloqueada y subir a target_z.
            # Usamos current_distance (rangefinder) como referencia de altura,
            # NO current_z, porque current_z proviene del barométrico mientras
            # el EKF2_HGT_REF=2 todavía no está convergido.
            setpoint.position = [safe_x, safe_y, self.target_z]
            error_alt = abs(self.current_distance - self.target_altitude)

            if error_alt < 0.40:
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0  # resetear si nos alejamos

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[TAKEOFF] dist={self.current_distance:.2f} m | "
                    f"err={error_alt:.2f} m | "
                    f"stable={self.stable_ticks}/{self.stable_ticks_needed} | "
                    f"xy_valid={self.xy_valid}"
                )

            if self.stable_ticks >= self.stable_ticks_needed:
                self.state = "HOLD"
                self.hold_start_time = None  # se inicializa en el siguiente estado
                self.get_logger().info(
                    f"TAKEOFF→HOLD: estable en {self.current_distance:.2f} m"
                )

        elif self.state == "HOLD":
            # Hover de 3 segundos para que el EKF2 converja el estimado XY
            # usando el H-Flow. Este tiempo es crítico: el filtro necesita
            # observar el flujo óptico estático para eliminar el bias inicial.
            setpoint.position = [safe_x, safe_y, self.target_z]

            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.hold_start_time
            ).nanoseconds / 1e9

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[HOLD] {elapsed:.1f}s / {self.hold_duration}s | "
                    f"dist={self.current_distance:.2f} m | "
                    f"xy_valid={self.xy_valid}"
                )

            if elapsed >= self.hold_duration:
                if self.xy_valid:
                    # Calcular objetivo: 2 m en la dirección en que mira el dron.
                    # En NED: adelante = [cos(yaw), sin(yaw)] en el plano XY.
                    self.forward_target_x = (
                        self.current_x + self.forward_distance * math.cos(safe_yaw)
                    )
                    self.forward_target_y = (
                        self.current_y + self.forward_distance * math.sin(safe_yaw)
                    )
                    self.state = "FORWARD"
                    self.get_logger().info(
                        f"HOLD→FORWARD: destino NED "
                        f"({self.forward_target_x:.2f}, {self.forward_target_y:.2f}) | "
                        f"yaw={math.degrees(safe_yaw):.1f}°"
                    )
                else:
                    # El H-Flow no está listo: esperar sin moverse.
                    # Causas posibles: calidad del sensor baja (superficie lisa,
                    # poca luz), parámetros EKF2_OF_CTRL o SENS_FLOW_* incorrectos.
                    self.get_logger().warn(
                        "HOLD: xy_valid=False → no se puede avanzar. "
                        "Verificar calidad del H-Flow y parámetros EKF2_OF_CTRL."
                    )
                    # Reiniciar el temporizador para volver a intentar en 3 s
                    self.hold_start_time = None

        elif self.state == "FORWARD":
            # Comandar el punto de destino calculado en HOLD.
            # El EKF2 (con H-Flow activo) estima la posición actual usando
            # el flujo óptico integrado; el controlador de posición de PX4
            # cierra el lazo hasta llegar al destino.
            setpoint.position = [
                self.forward_target_x,
                self.forward_target_y,
                self.target_z,
            ]

            # Distancia 2D al objetivo (en el plano XY del NED)
            dx = self.forward_target_x - self.current_x
            dy = self.forward_target_y - self.current_y
            dist_to_target = math.sqrt(dx * dx + dy * dy)

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[FORWARD] dist_restante={dist_to_target:.2f} m | "
                    f"pos=({self.current_x:.2f}, {self.current_y:.2f}) | "
                    f"xy_valid={self.xy_valid}"
                )

            # Seguridad: si el H-Flow pierde la señal durante el movimiento,
            # mantenemos el setpoint actual (PX4 lo sostendrá en hover).
            if not self.xy_valid and self.counter % 10 == 0:
                self.get_logger().warn(
                    "[FORWARD] xy_valid=False durante movimiento. "
                    "Manteniendo setpoint, esperando recuperación del sensor."
                )

            # Tolerancia de llegada: 0.25 m es razonable para flujo óptico interior.
            # Con GPS se podría bajar a ~0.10 m, pero el OF tiene más ruido.
            if dist_to_target < 0.25:
                self.state = "HOLD_FWD"
                self.hold_fwd_start_time = None
                self.get_logger().info(
                    f"FORWARD→HOLD_FWD: llegado, error residual={dist_to_target:.2f} m"
                )

        elif self.state == "HOLD_FWD":
            # Hover de 3 s sobre el punto de destino.
            setpoint.position = [
                self.forward_target_x,
                self.forward_target_y,
                self.target_z,
            ]

            if self.hold_fwd_start_time is None:
                self.hold_fwd_start_time = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.hold_fwd_start_time
            ).nanoseconds / 1e9

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[HOLD_FWD] {elapsed:.1f}s / {self.hold_duration}s | "
                    f"dist={self.current_distance:.2f} m | "
                    f"xy_valid={self.xy_valid}"
                )

            if elapsed >= self.hold_duration:
                self.state = "LAND"
                self.get_logger().info("HOLD_FWD→LAND: iniciando aterrizaje")

        elif self.state == "LAND":
            # Aterrizar sobre el punto de destino (no el punto de despegue).
            # Velocidad de descenso limitada a 0.4 m/s para que el rangefinder
            # tenga tiempo de detectar el suelo correctamente.
            land_x = self.forward_target_x if self.forward_target_x is not None else safe_x
            land_y = self.forward_target_y if self.forward_target_y is not None else safe_y

            setpoint.position = [land_x, land_y, 0.0]
            setpoint.velocity = [float('nan'), float('nan'), 0.4]  # m/s descenso suave

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[LAND] dist={self.current_distance:.3f} m | "
                    f"pos=({self.current_x:.2f}, {self.current_y:.2f})"
                )

            # El rangefinder del H-Flow detecta que tocamos el suelo
            if self.current_distance < 0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)  # MAV_CMD_COMPONENT_ARM_DISARM → desarmar
                self.get_logger().info("LAND→LANDED: suelo detectado, motores desarmados")

        elif self.state == "LANDED":
            # Estado terminal — no publicamos setpoints nuevos.
            pass

        # Publicar el setpoint en todos los estados activos
        if self.state != "LANDED":
            self.trajectory_pub.publish(setpoint)

        self.counter += 1

    # =========================================================================
    # UTILIDADES
    # =========================================================================

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PX4FlowPrecision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()