#########################################################################
# Imports:
#########################################################################

from threading import Event

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.timer import Rate, Timer
from rclpy.lifecycle import Node

from lifecycle_msgs.msg import State, Transition, TransitionEvent
from lifecycle_msgs.srv import GetState, ChangeState

#########################################################################
# Class:
#########################################################################

class ManagedNodeClient:
    def __init__(
        self,
        parent_node: Node,
        monitor_period_ms: int,
        node_name: str,
        node_namespace: str,
        **kwargs
    ):
        """
            Constructor of the ManagedNodeClient class.
        """
        
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

        self.cb_group_1 = MutuallyExclusiveCallbackGroup()
        
        self.get_state_client = parent_node.create_client(
            GetState,
            f'{self.long_node_name}/get_state',
            callback_group=self.cb_group_1
        )
        
        self.change_state_client = parent_node.create_client(
            ChangeState,
            f'{self.long_node_name}/change_state',
            callback_group=self.cb_group_1
        )

        self.transition_events_subscriber = parent_node.create_subscription(
            TransitionEvent,
            f'{self.long_node_name}/transition_event',
            self.transition_event_callback,
            10
        )

        self._state = self._request_state()

        if self._state is None:
            raise RuntimeError(f'Failed to get state of node "{self.long_node_name}".')
        
        self.monitor_timer = parent_node.create_timer(
            monitor_period_ms / 1000,
            self.monitor_callback
        )
        
    def _request_state(self) -> State:
        """
            Method for requesting the state of the managed node.
        """
        
        request = GetState.Request()
        
        if not self.get_state_client.wait_for_service(0.1):
            self.parent_node.get_logger().error(f'ManagedNodeClient._request_state(): Failed to get state of node "{self.long_node_name}", get_state server not responding.')
            return None

        event = Event()
        
        def future_callback(future):
            nonlocal event
            event.set()
        
        future = self.get_state_client.call_async(request)
        
        future.add_done_callback(future_callback)

        event.wait()
        
        if future.result() is not None:
            return future.result().current_state
        else:
            self.parent_node.get_logger().error(f'Failed to get state of node "{self.long_node_name}".')
            return None
    
    def _request_transition(
        self, 
        transition_id: int
    ) -> bool:
        """
            Method for requesting a transition of the managed node.
        """
        
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        if not self.change_state_client.wait_for_service(0.1):
            self.parent_node.get_logger().error(f'ManagedNodeClient._request_transition(): Failed to request transition of node "{self.long_node_name}", change_state server not responding.')
            return False
        
        event = Event()
        
        def future_callback(future):
            nonlocal event
            event.set()
        
        future = self.change_state_client.call_async(request)
        
        future.add_done_callback(future_callback)

        event.wait()
        
        return future.result() is not None

    def transition_event_callback(self, msg: TransitionEvent):
        """
            Callback for the transition event subscriber.
        """
        
        self._state = msg.goal_state
        
    def monitor_callback(self):
        """
            Callback for the monitor timer.
        """
        
        self._state = self._request_state()
        
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

    def _wait_for_state(
        self, 
        state_id: int,
        initial_state_id: int,
        timeout_ms: int = -1
    ) -> bool:
        """
            Method for waiting for the managed node to reach a certain state.
        """

        self.start_time = self.parent_node.get_clock().now()

        sleep_rate = self.parent_node.create_rate(10)
        
        while timeout_ms < 0 or (self.parent_node.get_clock().now() - self.start_time).nanoseconds / 1e6 < timeout_ms:
            self._state = self._request_state()
            
            if self._state.id == state_id:
                return True

            if self._state.id != initial_state_id:
                return False
            
            sleep_rate.sleep()
            
        return False

    def request_configure(self) -> bool:
        """
            Method for requesting the managed node to configure.
        """

        if not self._state.id == State.PRIMARY_STATE_UNCONFIGURED:
            return False
        
        if not self._request_transition(Transition.TRANSITION_CONFIGURE):
            return False
        
        return self._wait_for_state(
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_UNCONFIGURED
        )

    def request_activate(self) -> bool:
        """
            Method for requesting the managed node to activate.
        """

        if not self._state.id == State.PRIMARY_STATE_INACTIVE:
            return False
        
        if not self._request_transition(Transition.TRANSITION_ACTIVATE):
            return False
        
        return self._wait_for_state(
            State.PRIMARY_STATE_ACTIVE,
            State.PRIMARY_STATE_INACTIVE
        )
    
    def request_deactivate(self) -> bool:
        """
            Method for requesting the managed node to deactivate.
        """

        if not self._state.id == State.PRIMARY_STATE_ACTIVE:
            return False
        
        if not self._request_transition(Transition.TRANSITION_DEACTIVATE):
            return False
        
        return self._wait_for_state(
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_ACTIVE
        )
    
    def request_cleanup(self) -> bool:
        """
            Method for requesting the managed node to cleanup.
        """

        if not self._state.id == State.PRIMARY_STATE_INACTIVE:
            return False
        
        if not self._request_transition(Transition.TRANSITION_CLEANUP):
            return False
        
        return self._wait_for_state(
            State.PRIMARY_STATE_UNCONFIGURED,
            State.PRIMARY_STATE_INACTIVE
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
        
        return self._wait_for_state(
            State.PRIMARY_STATE_FINALIZED,
            initial_state_id
        )