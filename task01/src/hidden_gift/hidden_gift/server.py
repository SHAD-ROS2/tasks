"""ROS adapter supplied by the course; only terminal_kind is a core student task.
The execute coroutine waits on an rclpy Future, NOT on asyncio.sleep().
The 50 Hz timer advances the mission; callbacks never wait for robot motion.
"""
import math
import threading
import time
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from shad_interfaces.action import ExploreZone
from shad_interfaces.msg import SensorSample
from rclpy.qos import qos_profile_sensor_data
from course_lab.world import Area, Sample, Outcome
from .mission import Mission

@dataclass
class Active:
    handle: object
    future: Future
    mission: Mission
    started: float
    last_feedback: float = 0.0
    completing: bool = False

class GiftServer(Node):
    def __init__(self):
        super().__init__('gift_server')
        self.declare_parameter('task_timeout_sec',300.0)
        self.declare_parameter('sensor_timeout_sec',0.7)
        self.lock=threading.RLock()
        self.reserved=False
        self.active=None
        self.sample=None
        self.action_group=ReentrantCallbackGroup()
        self.io_group=MutuallyExclusiveCallbackGroup()
        self.pub=self.create_publisher(Twist,'cmd_vel',10)
        self.sub=self.create_subscription(SensorSample,'sample',self.on_sample,qos_profile_sensor_data,callback_group=self.io_group)
        self.health=self.create_service(Trigger,'health',self.on_health,callback_group=self.action_group)
        self.action=ActionServer(self,ExploreZone,'explore_zone',execute_callback=self.execute,
            goal_callback=self.on_goal,cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted,callback_group=self.action_group)
        self.timer=self.create_timer(0.02,self.tick,callback_group=self.io_group)

    def accept_request(self, area: Area) -> bool:
        # Provided infrastructure: input validation and atomic busy reservation.
        return area.valid() and not self.reserved

    def cancellation_policy(self):
        # Provided infrastructure. Cancellation is an optional L01 extension.
        return CancelResponse.ACCEPT

    def terminal_kind(self, outcome: Outcome) -> str:
        if outcome.kind == 'canceled': return 'canceled'
        if outcome.kind in ('found', 'absent'): return 'succeed'
        return 'abort'  # Faults stay ABORTED; starter does not falsely claim success.

    def guard(self, active: Active, now: float) -> Outcome | None:
        # Provided infrastructure; fault policy is covered in later lessons.
        if active.handle.is_cancel_requested:
            return Outcome('canceled',reason='cancel_requested')
        if now-active.started>float(self.get_parameter('task_timeout_sec').value):
            return Outcome('abort',reason='task_timeout')
        last=self.sample.received if self.sample is not None else active.started
        if now-last>float(self.get_parameter('sensor_timeout_sec').value):
            return Outcome('abort',reason='sensor_timeout')
        return None

    def on_goal(self, request):
        area=Area(request.min_x,request.min_y,request.max_x,request.max_y)
        with self.lock:
            if not self.accept_request(area): return GoalResponse.REJECT
            self.reserved=True  # Reserve atomically BEFORE handle_accepted.
            return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return self.cancellation_policy()

    def on_accepted(self, goal_handle):
        request=goal_handle.request
        with self.lock:
            self.active=Active(goal_handle,Future(),Mission(Area(request.min_x,request.min_y,request.max_x,request.max_y)),time.monotonic())
        goal_handle.execute()

    def on_sample(self, msg):
        p=msg.pose; c=msg.color
        with self.lock:
            self.sample=Sample(p.x,p.y,p.theta,(c.g==255 and c.r==0 and c.b==0),msg.sequence,time.monotonic())

    def on_health(self, request, response):
        with self.lock:
            response.success=True
            response.message='busy' if self.reserved else 'idle'
        return response

    def publish(self,v=0.0,w=0.0):
        msg=Twist(); msg.linear.x=float(v); msg.angular.z=float(w)
        self.pub.publish(msg)

    def tick(self):
        future=None; outcome=None
        with self.lock:
            run=self.active
            if run is None or run.completing: return
            now=time.monotonic()
            outcome=self.guard(run,now)
            if outcome is None and self.sample is not None:
                try:
                    d=run.mission.step(self.sample)
                    outcome=d.outcome
                    if outcome is None: self.publish(d.v,d.w)
                    if now-run.last_feedback>=0.2:
                        f=ExploreZone.Feedback(); f.distance_covered=float(d.distance)
                        run.handle.publish_feedback(f); run.last_feedback=now
                except Exception as e:
                    self.get_logger().error('Mission failed: '+str(e))
                    outcome=Outcome('abort',reason='mission_exception')
            if outcome is not None:
                self.publish()  # Stop before reporting terminal status.
                run.completing=True
                future=run.future
        if future is not None and not future.done(): future.set_result(outcome)

    async def execute(self, goal_handle):
        with self.lock: run=self.active
        outcome=await run.future
        with self.lock:
            # A cancellation accepted before terminal transition takes precedence.
            if goal_handle.is_cancel_requested: outcome=Outcome('canceled',reason='cancel_requested')
            self.publish()
            kind=self.terminal_kind(outcome)
            if kind=='succeed': goal_handle.succeed()
            elif kind=='canceled': goal_handle.canceled()
            else: goal_handle.abort()
            result=ExploreZone.Result()
            result.target_found=outcome.kind=='found' and kind=='succeed'
            result.target_x=float(outcome.x if result.target_found else 0.0)
            result.target_y=float(outcome.y if result.target_found else 0.0)
            self.active=None; self.reserved=False
            return result

def main(args=None):
    rclpy.init(args=args)
    node=GiftServer(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        try: node.publish()
        except Exception: pass
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
