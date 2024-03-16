# LoRa KISS AX.25 TNC

The codebase of this project seems to be based on the Work of [KC1AWV KISSLoRaTNC](https://github.com/kc1awv/KISSLoRaTNC)
If you know the real author, let me know.

I modified the code to be a compatible ax.25 Serial Kiss TNC and axUDP Sender/Receiver over WiFi.
Goal of this Project was to connect to my local Digipeater by "Packet Radio over LoRa" on 433 MHz.

The Digipeater Site is running [dxlAPRS Toolchain](https://github.com/oe5hpm/dxlAPRS/) for LoRa Rx/Tx and [XNet](http://xnet.swiss-artg.ch/) Packet Radio Node Software.

## Todo:

* Frame Splitting for Packets >255 Bytes
* Axudp Data validation before sending

