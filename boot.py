from mpy_env import load_env, get_env
import network

load_env()

if hasattr(network, "WLAN"):
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)

    wifi = get_env('wifi')

    if wifi is not None:
        if not sta_if.isconnected():
            print('connecting to network...')
            sta_if.active(True)
            sta_if.connect(wifi['ssid'], wifi['key'])
            while not sta_if.isconnected():
                pass
        print('network config:', sta_if.ifconfig())
    else:
        print('Wifi not in env')
else:
    print('Board is not wifi capable')