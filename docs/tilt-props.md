Ninjaflight Tilt Support
------------------------

For tilted propeller quads, ninjaflight provides yaw and roll compensation as
well as throttle compensation for cases where tilt is dynamic (ie controlled
through a radio channel). 

Commands
--------

- tilt [pitch|roll] motor [NONE|PITCH|ROLL|AUX1|AUX2] - this command sets the rc channel to be used
  for pitch tilt. If the channel is set to PITCH then rc pitch will control
  propeller tilt during flight and flight controller will try to stabilize the
  body (by assuming that requested pitch angle of the body is always static). 
- tilt [pitch|roll] body [NONE|AUX1|AUX2] - sets the channel to be used to control
  body pitch independently of the propeller pitch. This allows body to change
  pitch while leaving propeller pitch the same relative to earth frame. 
- tilt [pitch|roll] angle [value -maxangle to +maxangle] - sets static angle
  for either axis. Only used when mode is set to static.  
- tilt mode [dynamic|static|disabled] - sets the mode used for tilt. In dynamic
  mode the pitch and roll body/motor channels are used for controlling the
  tilting function. In static mode the "angle" setting is used as a static
  value for calculating flight compensation values. 
- tilt [pitch|roll] servorange [min] [max] - controlls the minimum and maximum
  angle of the propellers relative to earth frame (valid range -45 to 45 deg,
  min must be smaller than max). Note that this setting is also limited by
  physical characteristics of the frame design and the servo travel.  

Different flight modes
----------------------

The behaviour of the tilting logic is dependent on the flight mode that is
being used. 

- RATE mode: tilt angle must be controlled using one of the AUX channels. If
  one of the tilt channels is set to be controlled using pitch or roll stick
  then that function is ignored and middle value is used instead.  Flight
  controller compensates when yawing. Roll and pitch sticks tilt the multirotor
  as in normal rate mode. This mode is useful for tilting props into a certain
  angle and then keeping them tilted at the same angle during most of the
  flight. This mode can also be used with statically tilted props.  
- ANGLE mode: when pitch/roll is used as tilt channel, the controller will feed body
  tilt channel to the angle/rate controller as the amount of tilt to apply to
  the body and the pitch/roll channel controls tilting of the propellers. When
  body angle is set to zero or the channel is set to NONE, the quad essentially
  tilts the propellers to go forward and sideways while keeping the body stable
  in the air. 
- HORIZON mode: when sticks are around center position, the pitch/roll stick
  controls the tilt of the propellers. As stick moves to the extreme positions
  the body starts to tilt as well as it usually would do in normal RATE mode.
  This mode allows acro/racing flight.  

When in ANGLE or HORIZON modes the controller will try to level the propellers
regardless of the desired body angle so that the thrust is pointing upwards.  

Configuring for TILT Ranger
---------------------------

The TILT Ranger drone uses tilting propellers for pitch angle. Frame needs to
be set to "QuadX Tilt 1 Servo". The mode needs to be dynamic and pitch tilt
channel needs to be set to PITCH. Body channel can be left as default (NONE) or
set to one of the AUX channels. 

	tilt mode dynamic
	tilt pitch motor PITCH
	tilt roll motor NONE
	tilt pitch body NONE


