import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from play_motion2_msgs.action import PlayMotion2
import sys

class PlayMotionClient(Node):
    def __init__(self):
        super().__init__('play_motion_client')
        self.action_client = ActionClient(
            self,
            PlayMotion2,
            '/play_motion2',
        )
        self.get_logger().info('Play Motion Client has been started')

    def send_goal(self, motion_name, skip_planning=False):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # PlayMotion2 액션의 goal 메시지 생성
        goal_msg = PlayMotion2.Goal()
        goal_msg.motion_name = motion_name
        goal_msg.skip_planning = skip_planning

        self.get_logger().info(f'Sending goal: {motion_name} (skip_planning: {skip_planning})')

        # 비동기로 goal 전송
        self.send_goal_future = self.action_client.send_goal_async( # 요청에 대해 확인하는 future
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future): # 내가 보낸 goal에 대해서 액션 서버에서 승낙을 했는지 여부를 받는 콜백 함수
        goal_handle = future.result() # 결과가 아니라, 내가 요청한 goal에 대해서 승낙을 했는지에 대한 여부임
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected') # accepted가 아니면 액션 서버에서 거절당한 것 -> 결과 못 받음
            return

        self.get_logger().info('Goal accepted') # 액션 서버에서 승낙
        self.get_result_future = goal_handle.get_result_async() # 결과를 기다리기 위한 future
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result # result 안에 success, error 있음
        if result.success:
            self.get_logger().info('Motion completed successfully!')
        else:
            self.get_logger().error(f'Motion failed with error: {result.error}')

        self.get_logger().info('Action completed')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback # sec, nanosec
        # 현재 시간 출력 (초와 나노초)
        self.get_logger().info(
            f'Current time - sec: {feedback.current_time.sec}, '
            f'nanosec: {feedback.current_time.nanosec}'
        )

def main(args=None):
    rclpy.init(args=args)
    action_client = PlayMotionClient()

    # 명령줄 인자 처리
    motion_name = 'home'  # 기본값
    skip_planning = False  # 기본값

    if len(sys.argv) > 1:
        motion_name = sys.argv[1]
    if len(sys.argv) > 2:
        skip_planning = sys.argv[2].lower() == 'true'

    try:
        action_client.send_goal(motion_name, skip_planning)
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()