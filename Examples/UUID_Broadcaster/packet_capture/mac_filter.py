# import pyshark

# # Load the capture file
# capture = pyshark.FileCapture('capture.pcapng')

# # Set to store unique scanning MAC addresses
# unique_scanning_macs = set()

# for packet in capture:
#     if hasattr(packet, 'btle'):  # Check if the packet has the BTLE layer
#         btle_layer = packet.btle
#         # print(f"btle packet: {btle_layer}")
#         if hasattr(btle_layer, 'SCAN_REQ') and 'SCAN_REQ' in btle_layer.advertising_header_pdu_type:
#             scanning_address = btle_layer.scanning_address
#             # print(f"scanning_address: {scanning_address}")
#             unique_scanning_macs.add(scanning_address)

# # Display the unique scanning MAC addresses
# print("Unique Scanning MAC Addresses:")
# for mac in unique_scanning_macs:
#     print(mac)

import pyshark

# Load the capture file
capture = pyshark.FileCapture('capture.pcapng')

# Set to store unique scanning MAC addresses
unique_scanning_macs = set()

for packet in capture:
    # Check if the packet has the BTLE layer
    if hasattr(packet, 'btle'):
        btle_layer = packet.btle
        
        # Check if the PDU type is SCAN_REQ
        if hasattr(btle_layer, 'advertising_header_pdu_type') and '3' in btle_layer.advertising_header_pdu_type:# == '3':
            # Extract the scanning address
            scanning_address = btle_layer.get('scanning_address', None)
            if scanning_address:
                unique_scanning_macs.add(scanning_address)

# Print the unique scanning MAC addresses
print("Unique scanning MAC addresses:")
for mac in unique_scanning_macs:
    print(mac)


# import pyshark

# # Load the capture file
# capture = pyshark.FileCapture('capture.pcapng')

# for packet in capture:
#     if hasattr(packet, 'btle'):
#         btle_layer = packet.btle
#         print("Fields in BTLE layer:", btle_layer.field_names)
#         break