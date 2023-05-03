#!/usr/bin/python

# Import the relevant BLE Mesh Model classes for the Example VND Model from the ble_mesh_erpc_python package
from .ble_mesh_composition_data import BleMeshModelData, BleMeshModelApi, BleMeshModelCbk, BleMeshModelPub
from .lprf_erpc import lprf_api
from threading import Thread, Lock
from multiprocessing import Semaphore
import time

###################### BLE Mesh Generic OnOff Client Model ##########################################################


class GenericOnOffClientModel(BleMeshModelData):

    def __init__(self, element_index):
        """
            Init Generic OnOff Client Model attributes
        :param element_index: model element index
        """

        BleMeshModelData.__init__(self)

        # Init model api and cbk objects
        self.model_api = GenericOnOffClientAPI()
        self.model_cbk = GenericOnOffClientCbk()

        # Model opcodes
        self.opcodes = [[self.model_api.bt_mesh_model_op_2(0x82, 0x04), 0, "gen_onoff_status"]]

        # Init model attributes
        self.elem_index = element_index
        self.model_id = lprf_api.common.BT_MESH_MODEL_ID_GEN_ONOFF_CLI


class GenericOnOffClientAPI(BleMeshModelApi):

    def __init__(self):
        """
            Init the Generic OnOff Client Model Api attributes
        """

        BleMeshModelApi.__init__(self)
        self.onoff_state = 0

    def onoff_get(self, addr):
        """
            Send Get generic OnOff message
        :param addr: target message address
        """

        # Create and Init the OnOff Get message
        msg = self.net_buf_simple_raw([], 6, 0)
        self.bt_mesh_model_msg_init(msg=msg, opcode=self.bt_mesh_model_op_2(0x82, 0x01))

        # Init message ctx
        ctx = self.bt_mesh_msg_ctx_raw(net_idx=self.client_api.ble_mesh_prov_data.net_idx,
                                       app_idx=self.get_model_app_index(),
                                       addr=addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                       send_rel=0, send_ttl=0xff)

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_send_data(elem_idx=elem_idx, model_index=model_index, is_vnd=self.is_vnd(),
                                     ctx=ctx, msg=msg)

        return self.get_model_api_execute_rc()

    def onoff_set(self, addr, onoff, tid, transition_time=None, delay=None, ack=True):
        """
            Send Set generic OnOff message
        :param addr: target message address
        :param onoff: target OnOff state
        :param tid: transaction id
        :param transition_time: transition time, will sleep for (_total_duration * 1 ms)
        :param delay: delay, will sleep for (delay * 5 ms)
        :param ack: send ack status msg True/False
        """

        # Create and Init the OnOff Set message
        msg = self.net_buf_simple_raw([], 10, 0)
        if ack:
            self.bt_mesh_model_msg_init(msg=msg, opcode=self.bt_mesh_model_op_2(0x82, 0x02))
        else:
            self.bt_mesh_model_msg_init(msg=msg, opcode=self.bt_mesh_model_op_2(0x82, 0x03))

        # Add the msg data to the buffer
        self.net_buf_simple_add_u8(msg=msg, val=onoff)
        self.net_buf_simple_add_u8(msg=msg, val=tid)

        # Add transition time and delay to msg
        if transition_time is not None and delay is not None:
            self.net_buf_simple_add_u8(msg=msg, val=transition_time)
            self.net_buf_simple_add_u8(msg=msg, val=delay)

        # Init message ctx
        ctx = self.bt_mesh_msg_ctx_raw(net_idx=self.client_api.ble_mesh_prov_data.net_idx,
                                       app_idx=self.get_model_app_index(),
                                       addr=addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                       send_rel=0, send_ttl=0xff)

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_send_data(elem_idx=elem_idx, model_index=model_index, is_vnd=self.is_vnd(),
                                     ctx=ctx, msg=msg)

        return self.get_model_api_execute_rc()


class GenericOnOffClientCbk(BleMeshModelCbk):

    def __init__(self):
        """
            Init the Generic OnOff Client Model CBK attributes
        """

        BleMeshModelCbk.__init__(self)

    def gen_onoff_status(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            Generic OnOff Status Callback
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        cbk_msg = "Gen_OnOff_status from addr={:04x}".format(ctx[0].addr)
        data_len = len(buf[0].data)
        if data_len > 0:
            cbk_msg += ", present onoff={}".format(buf[0].data[0])
        if data_len == 3:
            cbk_msg += ", target onoff={}".format(buf[0].data[1])
            cbk_msg += ", remaining time={}".format(buf[0].data[2])
        self.print_msg(cbk_msg)


class GenericOnOffServerModel(BleMeshModelData):

    def __init__(self, element_index):
        """
            Init Generic OnOff Server Model attributes
        :param element_index: model element index
        """

        BleMeshModelData.__init__(self)

        # Init model api and cbk objects
        self.model_api = GenericOnOffServerAPI()
        self.model_cbk = GenericOnOffServerCbk()
        self.model_pub = BleMeshModelPub()
        self.model_pub.set_model_pub_raw(addr=0, key=0, ttl=0, retransmit=0, period=0, period_div=0,
                                         period_start=0, msg=self.model_api.net_buf_simple_raw([], 5, 0))

        # Model opcodes
        self.opcodes = [[self.model_api.bt_mesh_model_op_2(0x82, 0x01), 0, "gen_onoff_get"],
                        [self.model_api.bt_mesh_model_op_2(0x82, 0x02), 0, "gen_onoff_set"],
                        [self.model_api.bt_mesh_model_op_2(0x82, 0x03), 0, "gen_onoff_set_unack"]]

        # Init model attributes
        self.elem_index = element_index
        self.model_id = lprf_api.common.BT_MESH_MODEL_ID_GEN_ONOFF_SRV


class GenericOnOffServerAPI(BleMeshModelApi):

    def __init__(self):
        """
            Init the Generic OnOff Server Model Api attributes
        """

        BleMeshModelApi.__init__(self)

        # The model state
        self.present_onoff = 0

        # The set msg transaction ID
        self._tid = None

        # The target state
        self._target_onoff = 0

        # The transition time and _delay
        self._transition_time = 0
        self._delay = 0

        # Parameters that are used for the transition
        # time calculation
        self._remaining_time = 0
        self._total_duration = 0
        self._start_timestamp = 0

        # Flag that indicates if the last set msg should
        # get an acknowledged msg or not
        self._ack = False

        # The last msg parameters
        self._last_tid = None
        self._last_src_address = None
        self._last_dst_address = None
        self._last_msg_timestamp = None
        self._last_msg_ctx = None

        # State change transition thread and attributes
        self._state_change_transition_th = Thread(target=self._state_change_transition)
        self._state_change_transition_semaphore = Semaphore()
        self._state_change_transition_semaphore.acquire()
        self._run_state_change_transition = True
        self._state_change_transition_th.start()

        # A lock for the send_state_status msg
        self._state_lock = Lock()

    def __delete__(self):

        # Join the state_change_transition thread
        self._run_state_change_transition = False
        self._state_change_transition_semaphore.release()
        self._state_change_transition_th.join()

        BleMeshModelApi.__delete__(self)

    def _state_change_transition(self):
        """
            State Change Transition Thread Function ,
            This func use the delay clock for sleep (self._delay time * 5 ms)
            This func use the total duration clock for sleep (self._total_duration * 1 ms)
            0 -> 1 : delay sleep -> Update the present_onoff -> send_state_status -> total duration sleep -> publish_state_change
            1 -> 0 : delay sleep -> send_state_status -> total duration sleep -> Update the present_onoff -> publish_state_change
        """

        while True:

            # Wait for semaphore release to run the state change transition process
            self._state_change_transition_semaphore.acquire()

            # Check for stop Thread flag
            if not self._run_state_change_transition:
                break

            # Start the delay clock
            time.sleep(self._delay * 0.005)

            # Update the present state when the target is 1
            if self._target_onoff == 1:
                self.present_onoff = self._target_onoff

            # Send status only when needed
            if self._ack:
                self.send_state_status()

            # Reset to initial values
            self._delay = 0

            # Save the current timestamp
            self._start_timestamp = time.time()

            # Start the transition time clock
            time.sleep(self._total_duration * 0.001)

            # Update the present state when the target is 0
            if self._target_onoff == 0:
                self.present_onoff = self._target_onoff

            # Reset to initial values
            self._transition_time = 0
            self._remaining_time = 0
            self._total_duration = 0

            # Publish the new state
            self._publish_state_change()

    def _calculate_remaining_time(self):
        """
            Calculate the transition remaining time
        """

        resolution = 0
        steps = 0

        if self._start_timestamp == 0:
            duration_remainder = self._total_duration
        else:
            duration_remainder = self._total_duration - (time.time() - self._start_timestamp)

        if duration_remainder > 620000:
            # > 620 seconds -> resolution = 0b11 [10 minutes]
            resolution = 0x03
            steps = duration_remainder / 600000
        elif duration_remainder > 62000:
            # > 62 seconds -> resolution = 0b10 [10 seconds]
            resolution = 0x02
            steps = duration_remainder / 10000
        elif duration_remainder > 6200:
            # > 6.2 seconds -> resolution = 0b01 [1 seconds]
            resolution = 0x01
            steps = duration_remainder / 1000
        elif duration_remainder > 62000:
            # > 62 seconds -> resolution = 0b10 [10 seconds]
            resolution = 0x00
            steps = duration_remainder / 100

        self._remaining_time = (resolution << 6) | int(steps)

    def _set_transition_duration(self):
        """
            Set the transition duration
        """

        resolution = self._transition_time >> 6
        steps_multiplier = self._transition_time & 0x3F

        if steps_multiplier == 0:
            return

        if resolution == 0:
            self._total_duration = steps_multiplier * 100
        elif resolution == 1:
            self._total_duration = steps_multiplier * 1000
        elif resolution == 2:
            self._total_duration = steps_multiplier * 10000
        elif resolution == 3:
            self._total_duration = steps_multiplier * 600000

        self._remaining_time = time.time()

    def _publish_state_change(self):
        """
            Generic OnOff publish state change API, called when this model's OnOff state has changed
        """

        # Create and Init the OnOff present_onoff status message
        msg = self.net_buf_simple_raw([], 7, 0)
        self.bt_mesh_model_msg_init(msg=msg, opcode=self.bt_mesh_model_op_2(0x82, 0x04))

        # Add the current onoff state value
        self.net_buf_simple_add_u8(msg=msg, val=self.present_onoff)
        if self._total_duration != 0:
            # Add the target onoff value
            self.net_buf_simple_add_u8(msg=msg, val=self._target_onoff)
            # Add the remaining time value
            self.net_buf_simple_add_u8(msg=msg, val=self._remaining_time)

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_publish(elem_idx=elem_idx, is_vnd=self.is_vnd(), model_index=model_index, msg=msg)

        status_msg = "Publish Generic OnOff State Change"
        self.print_msg(status_msg)

    def send_state_status(self):
        """
            Generic OnOff send state status API, called when this model OnOff state is requested
        """

        # Create and Init the OnOff Get message
        msg = self.net_buf_simple_raw([], 9, 0)
        self.bt_mesh_model_msg_init(msg=msg, opcode=self.bt_mesh_model_op_2(0x82, 0x04))

        # Add the current onoff state value
        self.net_buf_simple_add_u8(msg=msg, val=self.present_onoff)
        if self._total_duration != 0:
            # Add the target onoff value
            self.net_buf_simple_add_u8(msg=msg, val=self._target_onoff)
            # Add the remaining time value
            self._calculate_remaining_time()
            self.net_buf_simple_add_u8(msg=msg, val=self._remaining_time)

        self._state_lock.acquire()

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_send_data(elem_idx=elem_idx, is_vnd=self.is_vnd(),
                                     model_index=model_index, ctx=self._last_msg_ctx, msg=msg)

        self._state_lock.release()

        status_msg = "Send Generic OnOff Status msg to addr={:04x}".format(self._last_msg_ctx.addr)
        self.print_msg(status_msg)

    def update_present_onoff(self, target_onoff):
        """ Update the present_onoff state """

        self.present_onoff = target_onoff

    def set_gen_onoff_state(self, ctx, buf, ack=False):
        """
            Generic OnOff set state API, called by the gen_onoff_set and gen_onoff_set_unack callbacks
        :param ctx: message ctx
        :param buf: message buffer
        :param ack: indicates if a status msg should be sent
        """

        # Save the tid
        self._tid = buf[0].data[1]

        # if the current msg tid, src address, dst address are the same as the previous msg
        # and it was received in the last 6 seconds, don't set the state, but return a status msg
        if self._last_tid == self._tid and self._last_dst_address == ctx.recv_dst and\
                self._last_src_address == ctx.addr and (time.time() - self._last_msg_timestamp < 6):
            if ack:
                self.send_state_status()
            return

        self._ack = ack

        # Save the target onoff state
        self._target_onoff = buf[0].data[0]

        # If transition time and delay exist, save the values
        if len(buf[0].data) == 4:
            # Save the transition time
            self._transition_time = buf[0].data[2]
            # Save the delay time
            self._delay = buf[0].data[3]

        # Save the last transaction parameters
        self._last_tid = self._tid
        self._last_msg_timestamp = time.time()
        self._last_src_address = ctx.addr
        self._last_dst_address = ctx.recv_dst
        self._total_duration = 0

        # No transition is needed if the present state and the target value is equal,
        # send a status msg
        if self.present_onoff == self._target_onoff:
            if ack:
                self.send_state_status()
            return

        # Set the transition duration
        else:
            self._set_transition_duration()

        # Save OnOff state to NV
        elem_index, model_index = self.get_model_indexes()
        self.bt_mesh_model_data_store(elem_index=elem_index, is_vnd=self.is_vnd(), model_index=model_index, name="onoff", data=[self._target_onoff])

        # If _total_duration value is 0, the transition is instantaneous
        self.print_msg("State Transition Total Duration = {}".format(self._total_duration))
        if self._total_duration == 0:
            self.present_onoff = self._target_onoff
            if ack:
                self.send_state_status()
            self._publish_state_change()
            return

        # Release the semaphore to start the state change transition process
        self._state_change_transition_semaphore.release()


class GenericOnOffServerCbk(BleMeshModelCbk):

    def __init__(self):
        """
            Init the Generic OnOff Server Model CBK attributes
        """

        BleMeshModelCbk.__init__(self)

    def gen_onoff_get(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            Generic OnOff Server Model Get callback, called when a Get message is received
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        self.print_msg("Gen_OnOff_get from addr={:04x}".format(ctx[0].addr))

        # Init message ctx
        self.model_api._last_msg_ctx = self.model_api.bt_mesh_msg_ctx_raw(net_idx=self.model_api.client_api.ble_mesh_prov_data.net_idx,
                                                                          app_idx=self.model_api.get_model_app_index(),
                                                                          addr=ctx[0].addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                                                          send_rel=0, send_ttl=0xFF)

        self.model_api.send_state_status()

    def gen_onoff_set(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            Generic OnOff Server Model Set callback, called when a Set message is received
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        # Verify that the onoff value is valid
        if buf[0].data[0] > 1:
            return

        cbk_msg = "Gen_OnOff_set from addr={:04x}, ".format(ctx[0].addr)
        cbk_msg += "OnOff={}, ".format(buf[0].data[0])
        cbk_msg += "TID={}".format(buf[0].data[1])
        if len(buf[0].data) == 4:
            cbk_msg += ", Transition Time={}".format(buf[0].data[2])
            cbk_msg += ", Delay={}".format(buf[0].data[3])
        self.print_msg(cbk_msg)

        # Init message ctx
        self.model_api._last_msg_ctx = self.model_api.bt_mesh_msg_ctx_raw(net_idx=self.model_api.client_api.ble_mesh_prov_data.net_idx,
                                                                          app_idx=self.model_api.get_model_app_index(),
                                                                          addr=ctx[0].addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                                                          send_rel=0, send_ttl=0xFF)

        self.model_api.set_gen_onoff_state(ctx[0], buf, ack=True)

    def gen_onoff_set_unack(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            Generic OnOff Server Model Set Unacknowledged callback, called when a Set Unacknowledged message is received
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        # Verify that the onoff value is valid
        if buf[0].data[0] > 1:
            return

        cbk_msg = "Gen_OnOff_set_unack from addr={:04x}, ".format(ctx[0].addr)
        cbk_msg += "OnOff={}, ".format(buf[0].data[0])
        cbk_msg += "TID={}".format(buf[0].data[1])
        if len(buf[0].data) == 4:
            cbk_msg += ", Transition Time={}".format(buf[0].data[2])
            cbk_msg += ", Delay={}".format(buf[0].data[3])
        self.print_msg(cbk_msg)

        self.model_api.set_gen_onoff_state(ctx[0], buf)

    def settings_set_model_cb(self, elem_idx, is_vnd, model_index, name, data):
        """
            Model settings_set_cb function
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param name: name of the data read from NV
        :param data: the data read from NV
        """

        self.print_msg("settings_set_model_cb: elem_idx={}, is_vnd={}, model_index={},"
                       "name={}, data={}".format(elem_idx, is_vnd, model_index, name, data))

        self.model_api.update_present_onoff(data[0])
