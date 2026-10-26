# State of the explorer_leds node, published on /explorer_leds/state when it changes (latched)

# Starting, the hardware isn't opened yet. Commands received meanwhile play once ready
uint8 INIT=0
# Hardware open, LEDs show the commanded effects
uint8 READY=1
# Invalid strip config (until restarted), or hardware that can't be opened or written (opening is
# retried continuously, back to READY once it succeeds)
uint8 ERROR=2

uint8 state

# Why the node is in ERROR. Empty otherwise
string message
