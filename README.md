Simple PID library tested using a dc motor.

** Functions **

pid_init: initialises a pid handler
pid_update: meant to be called at each timestep; returns new output
target_change: adjusts the target of the pid_handler; resets error_integral

** Safeguards **
1. Output clamping: prevent overloads of systems (e.g. a motor)
2. integral clamping: integral's contribution is held between same 
bounds as output to prevent pile ups
3. fabs on the gains: a negative gain won't flip a terms sign in the
final equation
4. Derivative on measurements: takes derivatives on measurements
to prevent large jumps when target value changes 