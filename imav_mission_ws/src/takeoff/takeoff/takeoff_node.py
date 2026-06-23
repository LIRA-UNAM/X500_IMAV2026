import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleControlMode,
)

# Constantes para la SM
SM_INIT = 0
SM_WAIT_CONFIRMATION = 10
SM_WAIT_LAUNCH = 20
SM_TAKEOFF = 30
SM_HOLD = 40
SM_ERROR = 50

# Parametros globales
CONFIMATION_TIMEOUT = 20.0
LOOP_RATE = 20
ALTITUDE_TOL = 0.20 # TOLERACIA DE ERROR
STABLE_TICKS_NEEDED = 20 # 20 Hz = 1 s

class PX4OffboardNode(Node):
    
    def local_pos_cb(self, msg: VehicleLocalPosition):
        # EKF2 Position
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading
    
    def control_mode_cb(self, msg: VehicleControlMode):
        # Monitoreo estado real de la Pix
        self.is_offboard = msg.flag_control_offboard_enabled
        self.is_armed = msg.flag_armed

    def takeoff_cb(self, msg: Bool):
        # Se espera la señal manual para poder despegar 
        if msg.data and self.start_takeoff and not self.launch_commanded:
            self.launch_commanded = True
            self.get_logger().info("Señal de despegue recibida.")
    
    def release_hold_cb(self, msg: Bool):
        # Espera recibir la señal del nodo externo para seguir con el vuelo
        if msg.data and self.takeoff_ready and not self.hold_released:
            self.hold_released = True
            self.get_logger().info("Señal recibida, dando control a nodo externo.")
    
    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        now = self.get_clock().now().nanoseconds // 1000

        msg                  = VehicleCommand()
        msg.timestamp        = now
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float):
        now = self.get_clock().now().nanoseconds // 1000

        ocm           = OffboardControlMode()
        ocm.timestamp = now
        ocm.position  = True
        self.offboard_pub.publish(ocm)

        sp              = TrajectorySetpoint()
        sp.timestamp    = now
        sp.position     = [float(x), float(y), float(z)]
        sp.velocity     = [float('nan'), float('nan'), float('nan')]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.jerk         = [float('nan'), float('nan'), float('nan')]
        sp.yaw          = float(yaw)
        sp.yawspeed     = float('nan')
        self.trajectory_pub.publish(sp)

    def __init__(self):
        super().__init__('px4_offboard_node')
        self.get_logger().info("INITIALIZING PX4 OFFBOARD NODE")

        # QoS configuration for PX4 uXRCE-DDS
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Suscribers
        self.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1', 
            self.local_pos_cb,
            sub_qos)
        
        self.create_subscription(VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.control_mode_cb,
            sub_qos)
        
        # Señal externa para disparar el despegue
        self.create_subscription(Bool,
            '/navigation/do_takeoff',
            self.takeoff_cb,
            1)
        
        # Señal externa para salir del HOLD y ceder control al nodo de aterrizaje
        self.create_subscription(Bool,
            '/navigation/release_hold',
            self.release_hold_cb,
            1)
        
        # Publishers
        self.offboard_pub    = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            pub_qos)
        
        self.trajectory_pub  = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            pub_qos)
        
        self.cmd_pub         = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            pub_qos)
        
        # takeoff_ready: True cuando el dron está estable en hold (1 m)
        self.takeoff_rdy_pub = self.create_publisher(
            Bool, 
            '/navigation/takeoff_ready',
            1)
        
        # Position Data (EKF2 Filtered)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        # State Variables
        self.is_offboard = False
        self.is_armed = False
        self.start_takeoff = False
        self.launch_commanded = False
        self.takeoff_ready = False
        self.hold_released = False

        # Mission Parameters
        self.target_altitude = 1.0          
        self.target_z = -self.target_altitude # NED coordinates: Z is negative

        # Helpers of the temporization
        self.confirmation_start = None
        self.stable_ticks = 0

    def spin(self):
        self.get_logger().info("INITIALIZED PX4 OFFBOARD NODE TO %d Hz" % LOOP_RATE)
        counter = 0
        state = SM_INIT

        while rclpy.ok():
            rclpy.spin_once(self)
            now = self.get_clock().now()

            # Lock starting position XY before takeoff
            if state in (SM_INIT, SM_WAIT_CONFIRMATION, SM_WAIT_LAUNCH):
                self.locked_x = self.current_x
                self.locked_y = self.current_y
                self.locked_yaw = self.current_yaw

            if state not in (SM_ERROR,) and not self.hold_released:
                self.publish_setpoint(self.locked_x, self.locked_y, self.current_z, self.locked_yaw)

            if state == SM_INIT:
                if counter >= LOOP_RATE:
                    self.get_logger().info("Solicitando modo OFFBOARD y ARMING")
                    self.send_cmd(176, param1=1.0, param2=6.0)
                    self.send_cmd(400, param1=1.0)
                    self.confirmation_start = now
                    state = SM_WAIT_CONFIRMATION
                
            elif state == SM_WAIT_CONFIRMATION:
                elapsed = (now - self.confirmation_start).nanoseconds / 1e9

                if self.is_offboard and self.is_armed:
                    self.start_takeoff = True
                    self.get_logger().info("Modo OFFBOARD y ARMING confirmados")
                    self.get_logger().info("Esperando señal de despegue externo")
                    state = SM_WAIT_LAUNCH

                elif elapsed >= CONFIMATION_TIMEOUT:
                    self.get_logger().info("FALLO DE COMUNICACIÓN")
                    state = SM_ERROR

                else:
                    if counter % LOOP_RATE == 0:
                        self.get_logger().info("Esperando confirmación de OFFBOARD y ARMING")
                        self.send_cmd(176, param1=1.0, param2=6.0)
                        self.send_cmd(400, param1=1.0)
                
            elif state == SM_WAIT_LAUNCH:
                if counter % (LOOP_RATE * 3) == 0:
                    self.get_logger().info("Esperando señal de despegue externo")
                
                if self.launch_commanded:
                    self.get_logger().info("Despegando a %.2f m" % self.target_altitude)
                    self.stable_ticks = 0
                    state = SM_TAKEOFF

            elif state == SM_TAKEOFF:
                self.publish_setpoint(self.locked_x, self.locked_y, self.target_z, self.locked_yaw)
                error_alt = abs(self.current_z - self.target_z)

                if error_alt < ALTITUDE_TOL:
                    self.stable_ticks += 1
                else:
                    self.stable_ticks = 0

                if counter % LOOP_RATE == 0:
                    self.get_logger().info(
                        "  TAKEOFF  |  altura=%.2f m  objetivo=%.2f m  "
                        "error=%.2f m  ticks=%d/%d"
                        % (abs(self.current_z), self.target_altitude,
                        error_alt, self.stable_ticks, STABLE_TICKS_NEEDED))
                    
                # Cambio de estado cuando se alcanza la estabilidad necesaria
                if self.stable_ticks >= STABLE_TICKS_NEEDED:
                    state = SM_HOLD
                    
            elif state == SM_HOLD:
                self.publish_setpoint(self.locked_x, self.locked_y, self.target_z, self.locked_yaw)
                
                if not self.takeoff_ready:
                    self.takeoff_ready = True
                    self.takeoff_rdy_pub.publish(Bool(data=True))
                
                if self.hold_released:
                    self.get_logger().info("Cediendo control a nodo externo")
                    break
                
                if counter % LOOP_RATE == 0:
                    self.get_logger().info("Esperando señal de ceder control a nodo externo")

            elif state == SM_ERROR:
                break

            counter += 1
            self.get_clock().sleep_for(Duration(seconds=1.0 / LOOP_RATE))

def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()