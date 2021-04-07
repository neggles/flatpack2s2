# Conventions in This Document
### **MSG** is a message from the power supply to the controller. 
### **CMD** is a message from the transceiver to the power supply.

---

# Hardware
The Flatpack2's CAN bus runs at 125kbps with 29-bit extended IDs, referenced to the PSU's negative rail.

If your breakout PCB exposes GNDD (the third small contact next to CANH and CANL), connect your transceiver GND to GNDD.  
If GNDD is not available, connect your transceiver GND to the supply's VOUT-.

**Failure to tie your transceiver's GND to GNDD or VOUT- will probably brutally murder your transceiver.**

If possible, use a CAN transceiver which can tolerate in excess of 55V bus fault voltage such as the MAX3305x and its 5V siblings; this makes transceiver damage less likely in the event of it being miswired.

I am using a MAX3051 in my application but the drop-in replacement MAX33053 would be a better idea.

---

# Notes

I can't confirm whether any of this protocol data is correct for models other than the Flatpack2 HE 48V/2000W and Flatpack2 HE 48V/3000W. That said, anecdotal evidence from posters on various forums suggests the Flatpack S models are command-compatible and will work as well.

I'd expect it to work with any black-fronted Flatpack2 (HE/SHE) or Flatpack S model, but am unable to confirm.

---

# ID-less Messages
These messages do not contain a PSU ID in their identity field, and are primarily used to discover PSUs and assign addresses to them.


## **CMD** Log in, `0x050048XX`
Log in to a power supply and assign it a CAN ID. Serial number payload chooses which supply to target.

PSU ID is assigned by setting `XX` to (ID * 4), e.g. sending message ID `0x05004804` assigns PSU ID of `0x01` for future commands.  

Allowable ID range is `0x01` to `0x3F`, or an `XX` of `0x04` through `0xFC`, and the power supply will log out if no login packet is received for (64 * 0.2) seconds.

<table>
    <thead>
        <tr>
            <th><b>Byte</b></th> <th>0</th> <th>1</th> <th>2</th> <th>3</th> <th>4</th> <th>5</th> <th>6</th> <th>7</th>
        </tr>
    </thead>
    <tr>
        <td><b>Value</b></td> <td colspan='6'>Power supply's serial number</td> <td><code>0x00</code></td> <td><code>0x00</code></td>
    </tr>
</table>

## **MSG** CAN hello packet, `0x0500XXXX`
A supply that is not logged in will send this packet every two seconds or so.

`XXXX` is the last two bytes of the PSU's serial number.

<table>
    <thead>
        <tr>
            <th><b>Byte</b></th> <th>0</th> <th>1</th> <th>2</th> <th>3</th> <th>4</th> <th>5</th> <th>6</th> <th>7</th>
        </tr>
    </thead>
    <tr>
        <td><b>Value</b></td> <td><code>0x1B</code></td> <td colspan='6'>Power supply's serial number</td> <td><code>0x00</code></td>
    </tr>
</table>

---

# PSU-specific Messages
These messages contain the PSU's assigned ID number, `XX`, in their message ID.

Sending messages to ID `0xFF` will command *all* power supplies on the bus - I think this only works for the 'Set voltage and current limits' command, untested.

At the moment I actually can't get single-supply commands to work at all, so that's... odd.


## **MSG** Status, `0x05XX40YY`
Once a power supply is logged in and has an address assigned, it will send these packets every 0.2 seconds exactly 64 times.

Transmit count is reset to 64 whenever a login is received; logging in every 10 seconds (or 5 to be safe) should be adequate to stay logged in.

Current is in deciAmps (A * 0.1), voltages are all in centiVolts (V * 0.01), all in little-endian byte order.


| Value of `YY` | Power supply state           |
| ------------- | ---------------------------- |
| `0x04`        | Normal (Constant Voltage)    |
| `0x08`        | Normal (Constant Current)    |
| `0x0C`        | Alarm                        |
| `0x10`        | Walk in (voltage ramping up) |


<table>
    <thead>
        <tr>
            <th><b>Byte</b></th> <th>0</th> <th>1</th> <th>2</th> <th>3</th> <th>4</th> <th>5</th> <th>6</th> <th>7</th>
        </tr>
    </thead>
    <tr>
        <td><b>Value</b></td> <td>Intake Temp</td> <td colspan='2'>Iout</td> <td colspan='2'>Vout</td> <td colspan='2'>Vin</td> <td>Exhaust Temp</td>
    </tr>
    <tr>
        <td><b>Unit</b></td> <td>°C</td> <td colspan='2'>dA (A * 0.1)</td> <td colspan='2'>cV (V * 0.01)</td> <td colspan='2'>Vrms (AC)</td> <td>°C</td>
    </tr>
</table>


## **MSG** Login request / start-up notification, `0x05XX4400`
Sent by a logged-out power supply every ten seconds or so. Similar to the CAN hello packet, but uses supply's pre-set ID.

<table>
    <thead>
        <tr>
            <th><b>Byte</b></th> <th>0</th> <th>1</th> <th>2</th> <th>3</th> <th>4</th> <th>5</th> <th>6</th> <th>7</th>
        </tr>
    </thead>
    <tr>
        <td><b>Value</b></td> <td colspan='6'>Power supply's serial number</td> <td><code>0x00</code></td> <td><code>0x00</code></td>
    </tr>
</table>


## **CMD** Set voltage and current limits, `0x05xx4004`
Sent to the power supply to immediately set output voltage and current limits.  
**If the supply logs out, these settings will be lost - default voltage will apply, and current limit will be set to factory maximum.**

Max current is the point at which the supply will switch from CV to CC modes.  
**Please note that the supply will not go below its minimum output voltage in CC mode!**

Desired voltage is the output voltage setpoint.

Measured voltage is for calibration/feedback; if you do not have a feedback voltage source, it should be equal to desired voltage.

OVP voltage is the voltage at which over-voltage protection will enable & cause the supply to shut down. Set this to your supply's maximum rated output voltage.

Max current is in deciAmps (A * 0.1), voltages are all in centiVolts (V * 0.01), all in little-endian byte order.

<table>
    <thead>
        <tr>
            <th><b>Byte</b></th> <th>0</th> <th>1</th> <th>2</th> <th>3</th> <th>4</th> <th>5</th> <th>6</th> <th>7</th>
        </tr>
    </thead>
    <tr>
        <td><b>Value</b></td> <td colspan='2'>Max Current</td> <td colspan='2'>Measured Voltage</td> <td colspan='2'>Desired Voltage</td> <td colspan='2'>OVP Voltage</td>
    </tr>
    <tr>
        <td><b>Unit</b></td> <td colspan='2'>dA (A * 0.1)</td> <td colspan='2'>cV (V * 0.01)</td> <td colspan='2'>cV (V * 0.01)</td> <td colspan='2'>cV (V * 0.01)</td>
    </tr>
</table>


## **CMD** Set default voltage, `0x05XX9C00`
Sent to the power supply to set its default voltage. Does not take effect until the supply is logged out. If the supply is logged in when the command is sent, the voltage is set when the log in times out. If it is not logged in, the voltage will be set when the supply logs in then times out.

Voltage is stored in the same format as the status message - centivolts (V * 0.01) in little-endian byte order.

<table>
    <thead>
        <tr>
            <th><b>Byte</b></th> <th>0</th> <th>1</th> <th>2</th> <th>3</th> <th>4</th>
        </tr>
    </thead>
    <tr>
        <td><b>Value</b></td> <td><code>0x29</code></td> <td><code>0x15</code></td> <td><code>0x00</code></td> <td colspan='2'>New voltage</td>
    </tr>
</table>


## **CMD** Alert request, `0x05XXBFFC`
Requests current alerts (warnings/alarms) from targeted power supply. 

Send this after receiving a status message with a last byte of `0x08` or `0x0C` to return current alert flags.

2nd byte of payload dictates whether query is for warning or critical alerts, and can be copied from the last byte of the status message.

| Byte  | 0    | 1                              | 2    |
| ----- | ---- | ------------------------------ | ---- |
| Value | 0x08 | 0x04 (warnings), 0x08 (alarms) | 0x00 |


## **MSG** Alert information, `0x05XXBFFC`
Response to CMD `0x05XXBFFC`, containing requested alert information.

2nd byte indicates whether flag bits are warning or critical, and is equal to 2nd byte of CMD packet.

| Byte  | 0    | 1                                | 2    | 3                 | 4                 | 5    | 6    |
| ----- | ---- | -------------------------------- | ---- | ----------------- | ----------------- | ---- | ---- |
| Value | 0x0E | 0x04 (Warning) / 0x08 (Critical) | 0x00 | Alert flag byte 1 | Alert flag byte 2 | 0x00 | 0x00 |


Alert flag bit mapping:

| Bit | Alert flag byte 1  | Alert flag byte 2  |
| --- | ------------------ | ------------------ |
| 0   | OVS Lock Out       | Internal Voltage   |
| 1   | Mod Fail Primary   | Module Fail        |
| 2   | Mod Fail Secondary | Mod Fail Secondary |
| 3   | High Mains         | Fan 1 Speed Low    |
| 4   | Low Mains          | Fan 2 Speed Low    |
| 5   | High Temp          | Sub Mod1 Fail      |
| 6   | Low Temp           | Fan 3 Speed Low    |
| 7   | Current Limit      | Inner Volt         |
