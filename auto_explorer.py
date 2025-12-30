import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random

class SmartPatrol(Node):
    def __init__(self):
        super().__init__('smart_patrol')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- LISTE DES POINTS A VISITER (PATROUILLE) ---
        # Ordre : Haut-Droite -> Bas-Droite -> Bas-Gauche -> Haut-Gauche -> Centre
        # Note : On met 2.0 et pas plus pour ne pas taper les murs du Stage 4
        self.waypoints = [
            (2.0, 2.0),   
            (2.0, -2.0),  
            (-2.0, -2.0), 
            (-2.0, 2.0),
            (0.0, 0.0)
        ]
        self.current_wp_index = 0
        self.patrol_mode = True # On commence par la patrouille forcée

        self.get_logger().info('Attente de Nav2...')
        self._action_client.wait_for_server()
        self.get_logger().info('C\'est parti ! Je commence la patrouille des coins.')
        self.send_next_goal()

    def send_next_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        target_x = 0.0
        target_y = 0.0

        if self.patrol_mode:
            # On prend le prochain point de la liste
            if self.current_wp_index < len(self.waypoints):
                target_x, target_y = self.waypoints[self.current_wp_index]
                self.get_logger().info(f'>>> PATROUILLE {self.current_wp_index+1}/{len(self.waypoints)} : Je vise le coin ({target_x}, {target_y})')
            else:
                # Liste finie, on passe en aléatoire
                self.patrol_mode = False
                self.get_logger().info('>>> PATROUILLE TERMINÉE ! Passage en mode aléatoire.')
                self.send_next_goal() # Relance immédiate
                return
        else:
            # Mode Aléatoire (pour finir le job)
            target_x = random.uniform(-2.2, 2.2)
            target_y = random.uniform(-2.2, 2.2)
            self.get_logger().info(f'>>> ALEATOIRE : Je vais vers ({target_x:.2f}, {target_y:.2f})')

        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.w = 1.0

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Mur détecté ou point inaccessible. Je passe au suivant.')
            # Si on est en patrouille et qu'il refuse un point, on le saute
            if self.patrol_mode:
                self.current_wp_index += 1
            self.send_next_goal()
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Arrivé ou échoué en cours de route
        if self.patrol_mode:
            self.current_wp_index += 1 # On valide l'étape
        
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    action_client = SmartPatrol()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
