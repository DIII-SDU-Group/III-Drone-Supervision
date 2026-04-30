"""Client helpers for interacting with managed lifecycle nodes.

`ManagedNodeClient` wraps the lifecycle service/action surface needed by the
supervision layer so the rest of the package can reason in terms of node state
and requested transitions instead of raw ROS calls.
"""

#########################################################################
# Imports:
#########################################################################

from threading import Event
from time import monotonic, sleep

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.timer import Rate, Timer
from rclpy.lifecycle import Node

from lifecycle_msgs.msg import State, Transition, TransitionEvent
from lifecycle_msgs.srv import GetState, ChangeState

from threading import Lock

#########################################################################
# Class:
#########################################################################

class ManagedNodeClient:
    def __init__(
        self,
        parent_node: Node,
        monitor_state: bool,
        monitor_period_ms: int,
        request_state_timeout_ms: int,
        node_name: str,
        node_namespace: str,
        monitor_callback_group: ReentrantCallbackGroup,
        **kwargs
    ):
        """
            Constructor of the ManagedNodeClient class.
        """
        
        self.monitor_state = monitor_state
        
        self.node_name = node_name
        if self.node_name[0] == '/':
            self.node_name = self.node_name[1:]
        if self.node_name[-1] == '/':
            self.node_name = self.node_name[:-1]
        
        self.node_namespace = node_namespace
        try:
            if self.node_namespace[0] == '/':
                self.node_namespace = self.node_namespace[1:]
            if self.node_namespace[-1] == '/':
                self.node_namespace = self.node_namespace[:-1]
        except IndexError:
            pass
                
        self.long_node_name = f'/{self.node_namespace}/{self.node_name}' if self.node_namespace != "" else f'/{self.node_name}'

        self.parent_node = parent_node
        
        self.monitor_callback_group = monitor_callback_group

        if not rclpy.ok():
            return
        
        self.get_state_client = self.parent_node.create_client(
            GetState,
            f'{self.long_node_name}/get_state',
            callback_group=self.monitor_callback_group
        )
        
        self.change_state_client = self.parent_node.create_client(
            ChangeState,
            f'{self.long_node_name}/change_state',
            callback_group=self.monitor_callback_group
        )

        # self.transition_events_subscriber = parent_node.create_subscription(
        #     TransitionEvent,
        #     f'{self.long_node_name}/transition_event',
        #     self.transition_event_callback,
        #     10
        # )

        self._request_state_timeout_ms = request_state_timeout_ms
        
        self._is_transitioning = False
        self._destroyed = False

        self._state: State = State()
        self._state.id = State.PRIMARY_STATE_UNKNOWN
        self._state.label = 'UNKNOWN'
        self._update_state()
        
        if not rclpy.ok():
            return

        # self.monitor_timer = parent_node.create_timer(
        #     monitor_period_ms / 1000,
        #     self.monitor_callback
        # )
        
    def _update_state(
        self,
        overwrite_timeout_ms: int = None
    ) -> State:
        """
            Method for updating the state of the managed node.
        """

        request = GetState.Request()
        if self._destroyed or getattr(self, "get_state_client", None) is None:
            return self._state
        
        timeout_ms = self._request_state_timeout_ms if overwrite_timeout_ms is None else overwrite_timeout_ms
        service_timeout_sec = 0.2
        if timeout_ms is not None and timeout_ms >= 0:
            service_timeout_sec = min(service_timeout_sec, timeout_ms / 1000)

        if not self.get_state_client.wait_for_service(service_timeout_sec):
            if self.monitor_state:
                if self._state is None or self._state.id != State.PRIMARY_STATE_UNKNOWN:
                    self.parent_node.get_logger().warn(f'ManagedNodeClient._request_state(): Failed to get state of node "{self.long_node_name}", get_state server not responding.')
                self._state = State()
                self._state.id = State.PRIMARY_STATE_UNKNOWN
                self._state.label = 'UNKNOWN'
            return self._state

        event = Event()
        
        def future_callback(future):
            nonlocal event
            event.set()

        future = self.get_state_client.call_async(request)
        
        future.add_done_callback(future_callback)

        start = monotonic()

        finished = False

        while rclpy.ok():
            finished = event.wait(0.1)
            
            if finished:
                break

            if timeout_ms >= 0 and (monotonic() - start) * 1000 > timeout_ms:
                break

        result: GetState.Response | None = future.result() if finished else None
        
        # with self.monitor_node_lock:
        #     future = self.get_state_client.call_async(request)
        
        #     rclpy.spin_until_future_complete(
        #         self.monitor_node, 
        #         future,
        #         timeout_sec=self._request_state_timeout_ms / 1000 if overwrite_timeout_ms is None else overwrite_timeout_ms / 1000
        #     )

        #     result: GetState.Response = future.result()

        #     if result is None:
        #         self.get_state_client.remove_pending_request(future)
        
        state: State = None
            
        if finished and result is not None:
            if self._state is not None and self._state.id == State.PRIMARY_STATE_UNKNOWN:
                self.parent_node.get_logger().info(f'Reacquired state of node "{self.long_node_name}".')
                
            state = result.current_state
        else:
            get_state_client = getattr(self, "get_state_client", None)
            if get_state_client is not None:
                get_state_client.remove_pending_request(future)
            if self.monitor_state:
                if rclpy.ok():
                    if self._state is not None and self._state.id != State.PRIMARY_STATE_UNKNOWN:
                        self.parent_node.get_logger().warn(f'Failed to get state of node "{self.long_node_name}".')
                state = State()
                state.id = State.PRIMARY_STATE_UNKNOWN
                state.label = 'UNKNOWN'
            
        if self.monitor_state or state is not None and state.id != State.PRIMARY_STATE_UNKNOWN:
            self._state = state
        
        return self._state

    def refresh_state(self, timeout_ms: int | None = None) -> State:
        """
            Public state refresh used before explicit lifecycle operations.
        """

        return self._update_state(overwrite_timeout_ms=timeout_ms)
    
    def _request_transition(
        self, 
        transition_id: int
    ) -> bool:
        """
            Method for requesting a transition of the managed node.
        """
        
        request = ChangeState.Request()
        if self._destroyed or getattr(self, "change_state_client", None) is None:
            return False

        request.transition.id = transition_id
        
        service_timeout_sec = min(0.5, self._request_state_timeout_ms / 1000)
        if not self.change_state_client.wait_for_service(service_timeout_sec):
            self.parent_node.get_logger().error(f'ManagedNodeClient._request_transition(): Failed to request transition of node "{self.long_node_name}", change_state server not responding.')
            return False
        
        event = Event()
        
        def future_callback(future):
            nonlocal event
            event.set()

        self._is_transitioning = True

        future = self.change_state_client.call_async(request)
        
        future.add_done_callback(future_callback)

        start = monotonic()
        finished = False

        while rclpy.ok():
            finished = event.wait(0.1)
            
            if finished:
                break

            if (monotonic() - start) * 1000 > self._request_state_timeout_ms:
                break

        result = future.result() if finished else None

        # with self.monitor_node_lock:
        #     # future = self.change_state_client.call(request)
        #     future = self.change_state_client.call_async(request)
        
        #     rclpy.spin_until_future_complete(
        #         self.monitor_node, 
        #         future,
        #         timeout_sec=self._request_state_timeout_ms / 1000.,
        #     )

        #     result = future.result()

        #     if result is None:
        #         self.change_state_client.remove_pending_request(future)
        #         return False
        
        if not finished or result is None:
            change_state_client = getattr(self, "change_state_client", None)
            if change_state_client is not None:
                change_state_client.remove_pending_request(future)
            self._is_transitioning = False
            return False
        
        self._is_transitioning = False

        if not result.success:
            self.parent_node.get_logger().error(
                f'ManagedNodeClient._request_transition(): Transition {transition_id} was rejected by node "{self.long_node_name}".'
            )
        
        return result.success

    def transition_event_callback(self, msg: TransitionEvent):
        """
            Callback for the transition event subscriber.
        """
        
        self._state = msg.goal_state
        
    def monitor_callback(self):
        """
            Callback for the monitor timer.
        """
        if self._destroyed:
            return

        self._update_state()
        
    @property
    def state(self) -> State:
        """
            Property for the state of the managed node.
        """
        
        return self._state

    @property
    def is_configured(self) -> bool:
        """
            Property for the configuration status of the managed node.
        """
        
        return self._state.id == State.PRIMARY_STATE_INACTIVE or self._state.id == State.PRIMARY_STATE_ACTIVE
    
    @property
    def is_active(self) -> bool:
        """
            Property for the activation status of the managed node.
        """
        
        return self._state.id == State.PRIMARY_STATE_ACTIVE

    def _verify_state_after_transition(self, state_id: int) -> bool:
        state = self.refresh_state(timeout_ms=self._request_state_timeout_ms)
        return state.id == state_id

    def _wait_for_state(
        self, 
        state_id: int,
        initial_state_id: int,
        ignore_state_ids: list = [],
        timeout_ms: int = -1
    ) -> bool:
        """
            Method for waiting for the managed node to reach a certain state.
        """

        # self.start_time = self.parent_node.get_clock().now()

        # sleep_rate = self.parent_node.create_rate(10)
        
        # while timeout_ms < 0 or (self.parent_node.get_clock().now() - self.start_time).nanoseconds / 1e6 < timeout_ms:
        deadline = monotonic() + (timeout_ms / 1000.0) if timeout_ms > 0 else None

        while rclpy.ok():
            self._update_state(overwrite_timeout_ms=timeout_ms if timeout_ms > 0 else None)
            
            if self._state.id == state_id:
                return True

            if self._state.id != initial_state_id and self._state.id not in ignore_state_ids:
                return False
            
            if deadline is not None and monotonic() >= deadline:
                break

            sleep(0.1)
            
        return False

    def request_configure(self) -> bool:
        """
            Method for requesting the managed node to configure.
        """

        if not self._state.id == State.PRIMARY_STATE_UNCONFIGURED:
            return False
        
        if not self._request_transition(Transition.TRANSITION_CONFIGURE):
            return False

        if not self.monitor_state:
            return self._verify_state_after_transition(State.PRIMARY_STATE_INACTIVE)
        
        return self._wait_for_state(
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_UNCONFIGURED,
            ignore_state_ids=[State.TRANSITION_STATE_CONFIGURING]
        )

    def request_activate(self) -> bool:
        """
            Method for requesting the managed node to activate.
        """

        if not self._state.id == State.PRIMARY_STATE_INACTIVE:
            return False
        
        if not self._request_transition(Transition.TRANSITION_ACTIVATE):
            return False
        
        if not self.monitor_state:
            return self._verify_state_after_transition(State.PRIMARY_STATE_ACTIVE)
        
        return self._wait_for_state(
            State.PRIMARY_STATE_ACTIVE,
            State.PRIMARY_STATE_INACTIVE,
            ignore_state_ids=[State.TRANSITION_STATE_ACTIVATING]
        )
    
    def request_deactivate(self) -> bool:
        """
            Method for requesting the managed node to deactivate.
        """

        if not self._state.id == State.PRIMARY_STATE_ACTIVE:
            return False
        
        if not self._request_transition(Transition.TRANSITION_DEACTIVATE):
            return False
        
        if not self.monitor_state:
            return self._verify_state_after_transition(State.PRIMARY_STATE_INACTIVE)
        
        return self._wait_for_state(
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_ACTIVE,
            ignore_state_ids=[State.TRANSITION_STATE_DEACTIVATING]
        )
    
    def request_cleanup(self) -> bool:
        """
            Method for requesting the managed node to cleanup.
        """

        if not self._state.id == State.PRIMARY_STATE_INACTIVE:
            return False
        
        if not self._request_transition(Transition.TRANSITION_CLEANUP):
            return False

        if not self.monitor_state:
            return self._verify_state_after_transition(State.PRIMARY_STATE_UNCONFIGURED)
        
        return self._wait_for_state(
            State.PRIMARY_STATE_UNCONFIGURED,
            State.PRIMARY_STATE_INACTIVE,
            ignore_state_ids=[State.TRANSITION_STATE_CLEANINGUP]
        )
    
    def request_shutdown(self) -> bool:
        """
            Method for requesting the managed node to shutdown.
        """

        initial_state_id = None

        if self._state.id == State.PRIMARY_STATE_UNCONFIGURED:
            transition = Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
            initial_state_id = State.PRIMARY_STATE_UNCONFIGURED
            
        elif self._state.id == State.PRIMARY_STATE_INACTIVE:
            transition = Transition.TRANSITION_INACTIVE_SHUTDOWN
            initial_state_id = State.PRIMARY_STATE_INACTIVE
            
        elif self._state.id == State.PRIMARY_STATE_ACTIVE:
            transition = Transition.TRANSITION_ACTIVE_SHUTDOWN
            initial_state_id = State.PRIMARY_STATE_ACTIVE
            
        else:
            return False
        
        if not self._request_transition(transition):
            return False

        if not self.monitor_state:
            return self._verify_state_after_transition(State.PRIMARY_STATE_FINALIZED)
        
        return self._wait_for_state(
            State.PRIMARY_STATE_FINALIZED,
            initial_state_id
        )

    def destroy(self) -> None:
        self._destroyed = True

        if not rclpy.ok():
            return

        if getattr(self, "get_state_client", None) is not None:
            self.parent_node.destroy_client(self.get_state_client)
            self.get_state_client = None

        if getattr(self, "change_state_client", None) is not None:
            self.parent_node.destroy_client(self.change_state_client)
            self.change_state_client = None
