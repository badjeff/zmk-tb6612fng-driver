# TB6612FNG Driver Module for ZMK

This module exposes TB6612FNG inputs via Zephyr's `sensor_driver_api` and key press behavior.

## Installation

Include this project on ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    #...
    - name: badjeff
      url-base: https://github.com/badjeff
    #...
  projects:
    #...
    - name: zmk-tb6612fng-driver
      remote: badjeff
      revision: main
    #...
  self:
    path: config
```

Update `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
/{
  /* setup the gpios to operate tb6612fng */
  tb6612fng_0: tb6612fng {
    compatible = "toshiba,tb6612fng";

    enable-gpios = <&gpio0 0 0 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
    
    ain1-gpios = <&gpio0 1 0 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
    ain2-gpios = <&gpio0 2 0 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;

    // bin1-gpios = <&gpio0 5 0 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
    // bin2-gpios = <&gpio0 5 0 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;

    /* map PWM gpios to AIN1/2 and BIN3/4 pins on tb6612fng */
    pwms = <&pwm 3 PWM_MSEC(20) PWM_POLARITY_NORMAL>
         //, <&pwm 4 PWM_MSEC(20) PWM_POLARITY_NORMAL>
         ;
  };
};
```

Update `board.keymap` for key press behavior:

```keymap
/{
    /* setup key press behavior, bind the tb6612fng driver */
    tb6612: behavior_tb6612fng_0 {
        compatible = "zmk,behavior-tb6612fng";
        #binding-cells = <2>;

        /* assign a tb6612fng device */
        tb6612fng-dev = <&tb6612fng_0>;

        /* define velocity scope of param2, e.g. range of 265, 128 as neutral point */
        vel-neutral = <128>;
        vel-min-max = <128>;
        // param1: channel = < 0 = IN1/2, 1 = IN3/4 >
        // param2: velocity = < min ... neutral ... max >
    };

    keymap {
        compatible = "zmk,keymap";
        default_layer {
                bindings = <
                    &tb6612 0 160 // channel 0, speed +32 (forward)
                    &tb6612 0 128 // channel 0, speed 0   (neutral)
                    &tb6612 0 96  // channel 0, speed -32 (reverse)
                >;
        };
    };
};
```

Enable the driver config in `<shield>.config` file (read the Kconfig file to find out all possible options):

```conf
# Enable PWM
CONFIG_PWM=y
CONFIG_PWM_LOG_LEVEL_DBG=y

# Enable debug logging
CONFIG_TB6612FNG_LOG_LEVEL_DBG=y
```
