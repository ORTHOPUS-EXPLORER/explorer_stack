// LED ring of the gripper, played in the page from the LED commands (explorer_msgs/msg/LedCommand)
// received on the websocket. Simplified look-alike of explorer_leds (led_animation.cpp): the icons
// show the first LEDs of the strip, effects on the other LEDs are ignored.

// Icons, clockwise from the top: icon i shows LED i of the commands
const LED_RING_ICON_COUNT = 4;
// Repeating modes are capped to this speed, as on the real LEDs
const LED_MAX_REPEATING_SPEED_HZ = 3.0;
const LED_REPEATING_MODES = new Set(['blink', 'alternate', 'breathe', 'chase']);
const LED_OFF = [0, 0, 0, 0];

// Colour [r, g, b, w] of a playing effect at nowSeconds. chase: position of the LED in the chase
function computeLedColor({ effect, start, chasePosition }, nowSeconds) {
    const colors = effect.colors || [];
    const color = (index) => {
        const intensity = (effect.intensities || [])[index] ?? 1;
        return (colors[index] || LED_OFF).map((component) => component * intensity);
    };
    const scaled = (factor) => color(0).map((component) => component * factor);

    let speed = effect.speed || 0;
    if (LED_REPEATING_MODES.has(effect.mode)) speed = Math.min(speed, LED_MAX_REPEATING_SPEED_HZ);
    // Cycles done since the effect started, the unit every mode is timed with
    const cycles = Math.max(nowSeconds - start, 0) * speed;

    switch (effect.mode) {
        case 'solid':
        case 'flash_once':
            return color(0);
        case 'fade_in':
            return scaled(Math.min(cycles, 1));
        case 'fade_out':
            return scaled(1 - Math.min(cycles, 1));
        case 'blink':
            return cycles % 1 < 0.5 ? color(0) : LED_OFF;
        case 'alternate':
            return colors.length ? color(Math.floor(cycles) % colors.length) : LED_OFF;
        case 'breathe': {
            const minLevel = effect.min_level || 0;
            const rise = (1 - Math.cos(2 * Math.PI * cycles)) / 2;
            return scaled(minLevel + (1 - minLevel) * rise);
        }
        case 'chase': {
            // The first colour moves one LED forward each cycle, the others trail behind it
            const length = effect.leds.length;
            const behindHead = (Math.floor(cycles) - chasePosition + length * 1e6) % length;
            return behindHead < colors.length ? color(behindHead) : LED_OFF;
        }
        default:
            return LED_OFF;
    }
}

class LedRing {
    constructor(container) {
        // Effect played by each LED, and a flash played once over it
        this.leds = Array.from({ length: LED_RING_ICON_COUNT }, () => ({ base: null, flash: null }));
        this.icons = this.leds.map((_, index) => {
            const icon = document.createElement('div');
            icon.className = `ring-led ring-led-${index}`;
            container.appendChild(icon);
            return icon;
        });
        requestAnimationFrame(() => this.render());
    }

    // Plays the effects of a LedCommand: in order, a later effect overriding an earlier one, the
    // LEDs no effect targets keeping theirs
    apply(effects) {
        const nowSeconds = performance.now() / 1000;
        for (const effect of effects || []) {
            // Empty: every LED
            const leds = effect.leds && effect.leds.length
                ? effect.leds
                : Array.from({ length: LED_RING_ICON_COUNT }, (_, led) => led);
            const playedEffect = { ...effect, leds };
            const key = JSON.stringify(playedEffect);

            leds.forEach((led, chasePosition) => {
                const state = this.leds[led];
                if (!state) return; // Not shown
                const playing = { effect: playedEffect, start: nowSeconds, chasePosition, key };
                if (effect.mode === 'flash_once') {
                    state.flash = playing;
                } else if (!state.base || state.base.key !== key) {
                    // The effect already playing isn't restarted
                    state.base = playing;
                    state.flash = null;
                }
            });
        }
    }

    render() {
        const nowSeconds = performance.now() / 1000;
        this.leds.forEach(({ base, flash }, index) => {
            const flashing = flash
                && (nowSeconds - flash.start) * (flash.effect.speed || 1) < 1;
            const playing = flashing ? flash : base;
            LedRing.paint(this.icons[index], playing ? computeLedColor(playing, nowSeconds) : LED_OFF);
        });
        requestAnimationFrame(() => this.render());
    }

    // Shows an [r, g, b, w] colour: hue at full brightness, as opaque as the LED is bright, over
    // the grey of a switched off LED
    static paint(icon, [r, g, b, w]) {
        const rgb = [r + w, g + w, b + w].map((component) => Math.min(component, 1));
        const brightness = Math.max(...rgb);
        if (brightness < 0.01) {
            icon.style.backgroundImage = '';
            icon.style.boxShadow = '';
            return;
        }
        const [red, green, blue] = rgb.map((component) => Math.round((component / brightness) * 255));
        const color = `rgba(${red}, ${green}, ${blue}, ${brightness.toFixed(3)})`;
        icon.style.backgroundImage = `linear-gradient(${color}, ${color})`;
        icon.style.boxShadow = `0 0 ${Math.round(4 + 6 * brightness)}px ${color}`;
    }
}
