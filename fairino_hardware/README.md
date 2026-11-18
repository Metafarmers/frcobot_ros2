# Fairino hardware: acknowledged ServoJ progress

Version 0.0.7 adds optional `servo_progress` GPIO handles for the first-party
`mf_manipulator_controllers/ProgressController`. Deploy the controller, hardware,
`mf_manipulator_msgs`, FR5 description and controller settings together.

Successful delayed ServoJ replies acknowledge one command without discarding the
trajectory. The next send requires fresh advancing feedback and pacing. The
driver checks finite commands and feedback, safety state and command ownership.
Manual and automatic modes are admitted. Safety and SDK faults still stop.
Without these GPIO handles, legacy timing protection remains enabled.

`servo_progress_resource` selects the GPIO resource name. The consumer's
`mf_manipulator/docs/servo-delay-recovery.md` describes the common contract,
ownership and deadlines.

`test_write_hold` compiles the real hardware path against test SDK symbols. Its
`RealJtc*` cases use the installed JTC action server and exercise successful
delays, consecutive delays, endpoint ACK and fixed goals on the Servo controller.
The tests cannot connect to or enable hardware. They are not HIL evidence.
