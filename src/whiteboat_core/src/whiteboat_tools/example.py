from common import MAVLinkConnection

# mav = MAVLinkConnection(baud=57600, simulating=False)
mav = MAVLinkConnection(sitl_address='udp:127.0.0.1:14550', simulating=True) # para usar via SITL
mav.connect()
mav.record_log()
