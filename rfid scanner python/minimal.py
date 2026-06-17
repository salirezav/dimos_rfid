from vulcan_rfid_reader import AdvanNetClient, AdvanNetEventStream, parse_event_xml

HOST = "192.168.123.2"
DEVICE = "VUL-TITANIUM-4PG-4e4e"

client = AdvanNetClient(HOST, "admin", "admin")
client.start(DEVICE)

with AdvanNetEventStream(HOST) as stream:
    for xml_msg in stream.messages():
        for tag in parse_event_xml(xml_msg):
            print(tag.epc, tag.rssi, tag.antenna)

client.stop(DEVICE)