# Animation played on some LEDs of the strip driven by explorer_leds

# Indices of the LEDs the effect applies to, from 0 to the strip LED count - 1.
# Empty means every LED
uint16[] leds

string MODE_OFF=off
string MODE_SOLID=solid
string MODE_FADE_IN=fade_in
string MODE_FADE_OUT=fade_out
string MODE_BLINK=blink
string MODE_ALTERNATE=alternate
string MODE_BREATHE=breathe
string MODE_FLASH_ONCE=flash_once

# One of the MODE_* constants:
#   off        : LEDs switched off, colors and intensities are ignored
#   solid      : steady colour (exactly one colour)
#   fade_in    : ramps from off up to the colour, then holds it (exactly one colour)
#   fade_out   : ramps from the colour down to off, then stays off (exactly one colour)
#   blink      : switches the colour on and off (exactly one colour)
#   alternate  : cycles through the colours in order (at least two colours)
#   breathe    : smoothly rises from off to the colour and back, repeatedly (exactly one colour)
#   flash_once : shows the colour once over the current animation, which then resumes
#                (exactly one colour). Meant to acknowledge an action without changing the status.
#                Only accepted on /explorer_leds/flash
string mode

# Animation speed in Hz, must be > 0 for every mode except off and solid:
#   fade_in / fade_out : 1 / fade duration
#   blink              : on/off cycles per second
#   alternate          : colour changes per second
#   breathe            : breaths per second
#   flash_once         : 1 / flash duration
# blink, alternate and breathe are capped at 3 Hz: faster flashing can trigger photosensitive seizures
float64 speed

# Colours to display, white included (see LedColor)
LedColor[] colors

# Brightness of each colour in [0, 1], one per colour: same size as colors
float64[] intensities
