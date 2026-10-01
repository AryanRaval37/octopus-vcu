# Stuff I need from you guys to finish the VCU code

The VCU logic runs and is tested in simulation, but a lot of the numbers in
it are guesses and some of the design depends on how the hardware ends up.
This is the list of things I still need. If something here is already
decided somewhere, just point me to it.

Anything with a number next to it is what the code uses right now as a
placeholder.

Rough priority: the BMS and inverter CAN docs are the big blocker. Without
them the VCU can't talk to anything. After that, the TS button question
and who switches the AIRs, because those change the state machine itself.


## BMS / accumulator (Manish)

- Which BMS is it? Off the shelf or our own?
- The CAN protocol doc, in writing. Message IDs, byte layout, endianness,
  scaling and offset for everything it sends. The stuff I actually use:
  pack voltage, min cell voltage, SOC, discharge current limit, charge
  current limit, max temperature, fault flag.
- Does it send a rolling counter and a checksum in its messages? If yes,
  which checksum algorithm. The rules (T11.9.2.d) want both corruption and
  message loss detected. If it doesn't have them and we can change the
  firmware, please add them. If we can't, I need to know soon so we can
  figure out what to show the scrutineers instead.
- How often each message is sent. The VCU times out the BMS after 200 ms,
  and at a 100 ms period that's only two missed frames. 50 ms or faster
  would be nicer. Otherwise I'll raise the timeout (the rules cap it at
  500 ms).
- Roughly how long after LV on before it sends its first message. I give it
  2 s right now before calling it missing.
- Are the current limits it sends instantaneous or continuous, and does it
  already lower them with temperature and SOC by itself?
- Does it send pack current on CAN? At what rate? The 80 kW limit is on
  electrical power, so the proper way to enforce it is to measure the
  current. Right now I'm guessing it from torque and an efficiency number.
- When it sees a cell fault, it opens the shutdown circuit itself (EV5.8
  says it has to). Does it also send a fault flag, and is that flag latched?
- Cell limits: at what min cell voltage should I start taking torque away
  (3300 mV) and when should there be none left (3000 mV)? Depends on the
  cells, so a datasheet is fine.
- Above what SOC should regen stop (95 %)?


## Inverter and motor

- Which inverter and which motor exactly (model numbers)?
- The inverter's CAN protocol doc. Mainly: how to send a torque command
  (units, scaling, sign), and what it reports (rpm, DC link voltage, motor
  and inverter temps, fault bits, actual torque, DC current if it has it).
- The enable sequence. A lot of inverters (AMK for example) need a specific
  order of "DC on", "enable", "inverter on" bits before they'll make
  torque. Right now the VCU only sets enable once we're in ready-to-drive.
  If it needs to be on earlier, for example during precharge, I need to
  know.
- Rolling counter and checksum in its messages? How often does it send?
  (inverter timeout is 100 ms)
- Max torque (200 Nm) and max rpm (6000).
- Efficiency of motor + inverter at high power. Even a rough worst case
  helps. I scale the 80 kW limit by this (90 %). If the real number is
  lower, we'd go over 80 kW at the energy meter, so it's better to guess low.
- At what motor and inverter temperatures should we start derating, and
  when should there be nothing left? (motor 90 -> 110 C, inverter 70 -> 85 C)
  Does the inverter already derate by itself?
- Are we doing regen at all? If yes, max regen torque (40 Nm). If not, I'll
  turn it off.
- Is the DC link voltage the inverter reports good enough to judge
  precharge (95 % of pack voltage)? Or is there a separate voltage
  measurement for that?
- Gear ratio and wheel radius. Not used yet, but needed for vehicle speed
  and anything like traction control later.


## Pedal box

- Part numbers for both APPS sensors.
- APPS2 is PWM, right? I need its frequency, the duty cycle range over full
  travel, and what it outputs when it's faulty or the wire's cut.
- Once the pedal box is built: the actual APPS1 voltage and APPS2 duty at
  fully released and fully pressed, measured on the car. Right now it's
  0.5 -> 4.5 V and 90 % -> 10 %, completely made up.
- Is there a pull-up or pull-down on the APPS1 signal line? If the wire gets
  cut, it has to land outside the normal range (near 0 V or near 5 V).
  Otherwise the VCU can't tell a cut wire from a real pedal position.
  Scrutineers will unplug each sensor to check this (T11.8.11).
- Which way each sensor moves as the pedal goes down.


## Brakes

- What are the two brake sensors? Two pressure sensors, one on the front
  circuit and one on the rear, or two sensors on the pedal itself? This one
  matters. The code currently treats them as two sensors measuring the same
  thing and complains if they differ by more than 15 %. If they're front and
  rear pressures, brake bias will make them differ all the time, and I need
  to change how they're checked.
- Part numbers and output range (pressure range and voltage range).
- What pressure should count as "brakes on"? This is used for entering
  ready-to-drive, the brake light, and the brake + throttle check. Right
  now it's 15 % of the sensor range.
- The BSPD trip points once it's trimmed: brake pressure and current. I
  want the software brake + throttle check to trip before the BSPD does,
  since a BSPD trip needs an LV power cycle.
- Is the BSPD reading the same sensors as the VCU, in parallel? T11.6.4
  says we'd have to show they don't interfere.


## Electrical / LV / harness

- Pin map for the VCU board: which S32K344 pins go to the APPS, brake
  sensors, buttons, relays, SDC sense, buzzer, brake light, CAN.
- ADC reference voltage and any dividers on the sensor inputs, so I can
  turn ADC counts into millivolts.
- Who actually switches AIR-, AIR+ and the precharge relay? The VCU, the
  AMS, or a separate precharge circuit? The code assumes the VCU does all
  three. If precharge is done in hardware, a chunk of the state machine
  becomes "watch it happen" instead of "make it happen".
- The TS on control in the cockpit: is it a switch that stays on, or a
  button you press once? The code assumes it stays on while the TS should
  be on (switching it off shuts down, and switching it off is also how the
  driver clears faults). If it's a momentary button, the state machine
  needs changing.
- Does the VCU have its own switch in the shutdown circuit (EV6.1.9 says
  it needs its own non-programmable power stage for that)? Where does the
  VCU sense the SDC, before or after its own switch? The code works either
  way, but I'd like to know.
- R2D button: momentary? Any hardware debounce?
- Is there a forward/neutral switch? If not I'll remove it from the code.
- Buzzer and brake light: does the VCU drive them directly? What's the load
  current?
- CAN: how many buses, which devices are on which, bit rate.
- Dash: what lights/display do we have and which ones does the VCU drive
  (fault light, R2D light, SOC...)?
- Cooling pump and fans: does the VCU control them? At what temperatures?
- Where should the datalog go: SD card on the VCU board, UART, or out over
  CAN to something else?


## Rules / scrutineering

Whoever's handling the ESF or talking to scrutineers, can you check these?
They're places where I had to pick an interpretation (more detail in
docs/design.md):

- APPS channels disagreeing for over 100 ms only cuts torque and leaves HV
  up (T11.8.8). An APPS wire actually broken opens the shutdown circuit and
  AIRs (T11.9.5).
- Brake + throttle check trips at 25 % throttle and resets under 5 %. The
  FB2027 rules require the check (A6.4.4) but don't give numbers anymore,
  so these are the old FSAE/FSG values.
- Ready-to-drive: the car goes live the moment the button is pressed with
  the brake held (and throttle released), and the 1.5 s sound plays from
  that moment. Not "sound first, then torque", because then the driver could
  be off the brake by the time the motors go live.
- Brake light also comes on during regen (T6.3.1).
- Is there an FB 2027 Q&A or FAQ that covers any of this?
