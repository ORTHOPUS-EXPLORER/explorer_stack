# Effects to play on the LEDs driven by explorer_leds, see LedEffect
#
# Status: published on /explorer_leds/command, once when it changes. The topic is latched:
# publish with TRANSIENT_LOCAL durability (explorer_leds::led_command_qos() in C++,
# --qos-durability transient_local with ros2 topic pub), so a LED node started later still gets the
# current status. A publisher with the default durability is incompatible and never received.
# Only the last command of each publisher is replayed to a LED node started later: a command must
# describe every LED its publisher controls, not only the ones that changed.
# Sending the effect already playing on a LED doesn't restart it.
#
# Events: flash_once is published on /explorer_leds/flash (default QoS), and only there, so it
# never replaces the latched status.
#
# Effects are applied in order, a later effect overriding an earlier one on the LEDs they share.
# A command with any invalid effect is ignored as a whole. LEDs no effect targets keep playing
# their current effect.
LedEffect[] effects
