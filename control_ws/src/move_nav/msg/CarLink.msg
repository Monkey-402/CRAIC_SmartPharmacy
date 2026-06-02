# 双车协调消息（车际 TCP 桥接与主控共用）
uint8 CARLINK_HEARTBEAT=0
uint8 CARLINK_SCAN_OK=1
uint8 CARLINK_ROUND_DONE=2
uint8 CARLINK_GO_HOME_ACK=3
uint8 CARLINK_NACK=4
uint8 CARLINK_REACHED_ABC=5

uint8 STATION_UNKNOWN=0
uint8 STATION_HOME=1
uint8 STATION_STANDBY=2
uint8 STATION_ON_MISSION=3

uint8 type
string from_id
uint32 seq
time stamp
uint8 station
int32 delivery_slot
