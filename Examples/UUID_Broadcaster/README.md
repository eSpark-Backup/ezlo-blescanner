
# UUID broadcaster
Broadcasts generic UUIDs over BLE indefinitely

Use BLE packet sniffer to capture BLE packets and filter out packets related to the devices mac address. Once packets are captured, use python script to filter out unique mac address that generated a response for the broadcasted UUIDs.
