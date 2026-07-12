std_msgs/Header header

int32 comm_state
int32[] id
bool[] torque_state
int32[] dxl_hw_state

int32 COMM_STATE_OK = 0
int32 COMM_STATE_CANNOT_FIND_CONTROL_ITEM = -1
int32 COMM_STATE_OPEN_PORT_FAIL = -2
int32 COMM_STATE_INDIRECT_ADDR_FAIL = -3
int32 COMM_STATE_ITEM_WRITE_FAIL = -4
int32 COMM_STATE_ITEM_READ_FAIL = -5
int32 COMM_STATE_SYNC_WRITE_FAIL = -6
int32 COMM_STATE_SYNC_READ_FAIL = -7
int32 COMM_STATE_SET_SYNC_WRITE_FAIL = -8
int32 COMM_STATE_SET_SYNC_READ_FAIL = -9
int32 COMM_STATE_BULK_WRITE_FAIL = -10
int32 COMM_STATE_BULK_READ_FAIL = -11
int32 COMM_STATE_SET_BULK_WRITE_FAIL = -12
int32 COMM_STATE_SET_BULK_READ_FAIL = -13
int32 COMM_STATE_SET_READ_ITEM_FAIL = -14
int32 COMM_STATE_SET_WRITE_ITEM_FAIL = -15
int32 COMM_STATE_DXL_HARDWARE_ERROR = -16
int32 COMM_STATE_DXL_REBOOT_FAIL = -17

# dxl_hw_state = 0: no error
# dxl_hw_state = 1: input voltage error
# dxl_hw_state = 4: overheating
# dxl_hw_state = 8: motor encoder
# dxl_hw_state = 16: electrical shork
# dxl_hw_state = 32: Overload
