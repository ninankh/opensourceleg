# Raspberry Pi Setup Guide (VSO / opensourceleg)

## 1. Hardware overview

| Function                           | Part                                               | Interface  | Pins / device node                                                                                      |
| ---------------------------------- | -------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------- |
| Quadrature encoder counter         | LS7366R                                            | SPI0       | CS = GPIO8 (CE0), `/dev/spidev0.0`                                                                      |
| ADC (Hall sensor, battery voltage) | ADS114S08                                          | SPI1       | CS1 = GPIO17, `/dev/spidev1.1`, DRDY = GPIO16                                                           |
| Hall-effect sensor                 | DRV5056                                            | via ADC    | ADC input AIN5 (verify wiring)                                                                          |
| Motor driver                       | VNH7070AY H-bridge driving the Maxon brushed motor | PWM + GPIO | GPIO12 (PWM0), GPIO24 (SEL0 + INA), GPIO25 (INB), GPIO26 (SEL1), MOT_IS current sense (see section 2.1) |
| IMU #1                             | external sensors                                   | UART0      | GPIO14 (TX) / GPIO15 (RX), `/dev/ttyAMA0`                                                               |
| IMU #2                             | external sensors                                   | UART2      | GPIO4 (TX) / GPIO5 (RX), `/dev/ttyAMA2`                                                                 |
| Analog switch                      | ADG852                                             | GPIO       | IN pin on GPIO39 for mux1 and GPIO35 for mux0, driven low at boot                                       |

ADG852 logic: IN high connects S1 to D; IN low connects S2 to D. With GPIO39 low at boot, S2 is connected to D. The switch is break-before-make.

---

## 2. Boot configuration

File: `/boot/firmware/config.txt` (older images: `/boot/config.txt`)

```
sudo nano /boot/firmware/config.txt
```

```ini
[all]
dtparam=spi=on                  # SPI0: MOSI=GPIO10, MISO=GPIO9, SCLK=GPIO11, CE0=GPIO8 (LS7366R)

# SPI1 for the ADS114S08 ADC (CS1 = GPIO17, DRDY = GPIO16 stays free)
dtoverlay=spi1-2cs

# ADG852 IN pins: drive low at boot (S2 connected to D)
gpio=39=op,dl                   # mux1
gpio=35=op,dl                   # mux0

# Hardware PWM0 on GPIO12 for the VNH7070AY motor driver
dtoverlay=pwm,pin=12,func=4

[cm5]
dtoverlay=i2c3-pi5,pins_22_23
gpio=78=op,dh                   # CM5 pin 78 (GPIO_VREF), enables the 1.8V LDO
dtoverlay=uart0-pi5             # GPIO14/15 -> /dev/ttyAMA0, first IMU
dtoverlay=uart2-pi5             # GPIO4/5   -> second IMU
```

Reboot after any change:

```
sudo reboot
```

### What each line does

| Line                            | Purpose                                                                                                                                                                                          |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `dtparam=spi=on`                | Enables SPI0. GPIO8 (CE0) is the motor driver chip select. This also claims GPIO7 (CE1). If CE1 is not needed, use `dtoverlay=spi0-1cs` instead (never both).                                    |
| `dtoverlay=spi1-2cs`            | Enables SPI1 with two chip selects: CE0 = GPIO18, CE1 = GPIO17. Creates `/dev/spidev1.0` and `/dev/spidev1.1`. Do **not** use `spi1-3cs`: it claims GPIO16 (CE2), which is needed for ADC DRDY.  |
| `gpio=39=op,dl`                 | Sets GPIO39 as an output, driven low, very early in boot. Controls the ADG852 IN pin. Add a 10 kOhm external pull-down on IN if the switch must never be in the wrong state at power-up.         |
| `gpio=35=ip,pd`                 | Sets GPIO35 as an input with an internal pull-down. Use `gpio=35=op,dl` to actively drive it low instead. Internal pull-downs are weak, so use an external resistor if a firm default is needed. |
| `dtoverlay=dwc2,dr_mode=host`   | USB controller in host mode.                                                                                                                                                                     |
| `dtoverlay=i2c3-pi5,pins_22_23` | Enables I2C3 on GPIO22/23.                                                                                                                                                                       |
| `gpio=78=op,dh`                 | CM5 pin 78 (`GPIO_VREF`), not a GPIO number. Drives the pin high to enable the 1.8V LDO.                                                                                                         |
| `dtoverlay=uart0-pi5`           | Enables UART0 on GPIO14/15 (`/dev/ttyAMA0`) for the first IMU.                                                                                                                                   |
| `dtoverlay=uart2-pi5`           | Enables UART2 on GPIO4/5 for the second IMU.                                                                                                                                                     |

### UART notes (Pi 5 / CM5)

- `/dev/ttyAMA10` is the dedicated debug console UART (`console=ttyAMA10,115200`). Do **not** use it for IMU data.
- `/dev/ttyAMA0` is the GPIO14/15 UART used for IMU #1.
- Avoid `uart1-pi5` (conflicts with HAT ID pins) and `uart3-pi5` (overlaps SPI0, which the motor driver uses).
- UART lines idle high. `pinctrl get 14` showing `lo` means something is wrong: wrong config, short to ground, or TX/RX wiring swapped.

### Verify after reboot

```
pinctrl get 8 9 10 11 14 15 16 17 18 35 39
ls /dev/spidev0.* /dev/spidev1.* /dev/ttyAMA*
gpioinfo | grep -E "line +(16|17|18):"
```

Expected:

| Pin             | Expected `pinctrl` output         |
| --------------- | --------------------------------- |
| GPIO8-11        | SPI0 alternate function           |
| GPIO14 / GPIO15 | `a4` (UART0), TX idles `hi`       |
| GPIO16          | `ip` (plain input, free for DRDY) |
| GPIO17 / GPIO18 | SPI1 CS functions                 |
| GPIO35          | `ip pd`                           |
| GPIO39          | `op dl`                           |

Device nodes `/dev/spidev0.0`, `/dev/spidev1.0`, `/dev/spidev1.1` should exist.

---

## 3. SSH key for GitHub

Generate a key on the Pi (replace the email with your GitHub email):

```
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Press Enter to accept the default path (`~/.ssh/id_ed25519`). A passphrase is recommended.

Start the agent and add the key:

```
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

Copy the **public** key (never share the private key, the file without `.pub`):

```
cat ~/.ssh/id_ed25519.pub
```

Add it on GitHub:

1. GitHub, then Settings, then **SSH and GPG keys**, then **New SSH key**.
2. Give it a title (for example `rpi-vso`), key type **Authentication Key**.
3. Paste the public key and save.

Test the connection:

```
ssh -T git@github.com
```

You should see a message like `Hi <username>! You've successfully authenticated`.

Use SSH URLs for repositories:

```
git clone git@github.com:<owner>/opensourceleg.git
# or, for an existing HTTPS clone:
git remote set-url origin git@github.com:<owner>/opensourceleg.git
```

Set your git identity once:

```
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

---

## 4. Getting started with the script

### 4.1 Get the code

```
cd ~
git clone git@github.com:<owner>/opensourceleg.git
cd opensourceleg
```

### 4.2 Python environment and dependencies

```
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

If pip refuses to install system-wide on Bookworm, use the venv above (or add `--break-system-packages` at your own risk).

Libraries used by the hardware classes (install any that are missing): `spidev`, `gpiozero`, `lgpio`, `numpy`, `pyserial`.

### 4.3 Permissions

```
sudo usermod -aG spi,gpio,dialout $USER
```

Log out and back in for group changes to apply. This gives access to `/dev/spidev*`, GPIO and serial ports.

### 4.4 Check the hardware first

```
pinctrl get 16 17 18
ls -l /dev/serial* /dev/ttyAMA*
ls /dev/spidev*
```

Use the verification table in section 2 if anything looks off.

### 4.5 Run the sensor script

```
cd ~/opensourceleg
source .venv/bin/activate
python tutorials/sensors/reading_vso_sensors.py
```

What it reads:

- **DRV5056 Hall sensor** through the ADS114S08 ADC. SPI1, `spi_cs=1` (GPIO17), DRDY on GPIO16.
- **AS5048B ankle encoder.**
- ADC voltage reference is about 1.68 V on this board, set through the shared `ADC_REFERENCE_VOLTAGE` constant. A hardcoded 1.65 V gives wrong readings.

### 4.6 Control loop frequencies (for reference)

| Layer                                          | Value                                           |
| ---------------------------------------------- | ----------------------------------------------- |
| Control loop (`SoftRealtimeLoop`, `FREQUENCY`) | 200 Hz                                          |
| MaxonActuator internal frequency               | 10 kHz                                          |
| LS7366R SPI clock (`SPI_MAX_HZ`)               | 1 MHz, mode 0 (CPOL=0, CPHA=0), 8 bits per word |

---

## 5. Troubleshooting

### "GPIO busy" error

Cause: a previous run is still holding the pin (usually GPIO16, DRDY). This often happens when the script was suspended with **Ctrl-Z** instead of exited with **Ctrl-C**.

```
jobs                                  # suspended jobs in this terminal
ps aux | grep -i python               # find leftover processes
sudo lsof /dev/gpiochip*              # who has the GPIO device open
kill <PID>                            # or: kill -9 <PID>
```

Always stop scripts with **Ctrl-C**.

### SPI1 chip select pins show as plain outputs

`pinctrl get 17` / `18` showing `op` instead of an SPI function means something claimed them after boot. Check:

```
gpioinfo | grep -E "line +(17|18):"
```

Consumers `spi1 CS1` / `spi1 CS0` are correct. A consumer of `lg` or a Python process name means user code grabbed the pin.

### ADC reads a stuck value (about 1.62 V, no response to magnet)

1. Confirm the channel mapping: switch the channel to AIN0 (known good from the battery-voltage code) and see if the reading changes.
2. With a multimeter, check the DRV5056 OUT pin against the ADC AIN5 pin to confirm they are electrically connected.
3. Confirm the reference voltage constant matches the board (about 1.68 V).

### Hall sensor always returns 0

Check that the `DRV5056.voltage` property is implemented (not a stub returning 0.0), and that offset and scale are in the same units (volts vs millivolts).

### UART issues

```
pinctrl get 14
pinctrl get 15
ls -la /dev/serial* /dev/ttyAMA*
dmesg | grep -i -E "uart|tty"
```

GPIO14/15 should show `a4`. If TX reads `lo` when idle, check wiring (RX/TX swapped), shorts on the PCB, or a faulty device.

### Config change had no effect

- Confirm you edited `/boot/firmware/config.txt` (not `/boot/config.txt`) on Bookworm.
- Confirm the line is under the right section: `[all]` applies to every board, `[cm5]` only to CM5.
- Reboot.

---

## 6. Quick reference

```
sudo nano /boot/firmware/config.txt   # edit boot config
sudo reboot                           # apply changes
pinctrl get <gpio>                    # pin function and level
pinctrl set <gpio> op dl              # set output low (until reboot)
gpioinfo                              # who owns each GPIO line
ls /dev/spidev* /dev/ttyAMA*          # device nodes
ssh -T git@github.com                 # test GitHub SSH
```
