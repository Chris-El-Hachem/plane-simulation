extends RigidBody3D

# Major credit to Vazgriz
# https://www.youtube.com/watch?v=7vAHo2B1zLc
# https://github.com/vazgriz/FlightSim

var wind: Vector3 = Vector3.RIGHT * 0

var max_thrust: float = 5000

var min_drag_coefficient: float = 1
var max_drag_coefficient: float = 3

@export var lift_curve: Curve
var base_angle_of_attack: float = 4 # deg
var lift_power: float = 30

var yaw_force_power: float = 5

var angular_drag_coefficient: float = 1

var weathervane_torque_coefficient: float = 50

var turning_speed: float = 1 # rad/s
var steering_responsiveness: float = 50000

var max_banking_angle: float = 60 # deg

var anti_roll: bool = true

func _physics_process(delta: float) -> void:
	var yaw_input: float = Input.get_axis("yaw left", "yaw right")
	var pitch_input: float = Input.get_axis("pitch down", "pitch up")
	var roll_input: float = Input.get_axis("roll left", "roll right")
	var thrust_input: float = Input.get_action_strength("thrust")
	
	if Input.is_action_just_pressed("anti-roll toggle"):
		anti_roll = !anti_roll
		print("Anti-Roll: " + str(anti_roll))
	
	var thrust_direction: Vector3 = -basis.z
	var thrust_magnitude: float = thrust_input * max_thrust
	var thrust: Vector3 = thrust_magnitude * thrust_direction
	apply_central_force(thrust)
	
	var local_air_velocity: Vector3 = wind - linear_velocity
	var drag_direction: Vector3 = local_air_velocity.normalized()
	var drag_correspondence: float = drag_direction.dot(-basis.z)
	var drag_coefficient: float = remap(drag_correspondence, -1, 1, min_drag_coefficient, max_drag_coefficient)
	var drag: Vector3 = drag_coefficient * local_air_velocity.length_squared() * drag_direction
	apply_central_force(drag)
	
	var lift_direction: Vector3 = basis.y
	var lift_air_flow: Vector3 = global_basis.inverse() * local_air_velocity
	lift_air_flow.x = 0
	var angle_of_attack: float = rad_to_deg(atan2(lift_air_flow.y, lift_air_flow.z))
	var lift_coefficient: float = lift_curve.sample(angle_of_attack + base_angle_of_attack)
	var lift: Vector3 = lift_power * lift_coefficient * lift_air_flow.length_squared() * lift_direction
	apply_central_force(lift)
	
	var yaw_force_direction: Vector3 = basis.x
	var yaw_force_air_flow: Vector3 = global_basis.inverse() * local_air_velocity
	yaw_force_air_flow.y = 0
	var yaw_angle_of_attack: float = yaw_input * 6 + rad_to_deg(atan2(yaw_force_air_flow.x, yaw_force_air_flow.z))
	var yaw_force_coefficient: float = lift_curve.sample(yaw_angle_of_attack)
	var yaw_force: Vector3 = yaw_force_power * yaw_force_coefficient * yaw_force_air_flow.length_squared() * yaw_force_direction
	apply_central_force(yaw_force)
	
	# This torque acts to reduce the angular velocity
	var angular_drag: Vector3 = angular_drag_coefficient * angular_velocity.length_squared() * -angular_velocity.normalized()
	apply_torque(angular_drag)
	
	# This torque pushes the plane to directly face air flow
	var weathervane_torque: Vector3 = basis.z.cross(local_air_velocity.normalized()) * local_air_velocity.length_squared() * weathervane_torque_coefficient
	apply_torque(weathervane_torque)
	
	var world_angular_velocity: Vector3 = global_basis.inverse() * angular_velocity
	var desired_angular_velocity: Vector3
	if anti_roll:
		desired_angular_velocity = Vector3(pitch_input, 0, 0).normalized() * turning_speed 
	else:
		desired_angular_velocity = Vector3(pitch_input, -yaw_input, -roll_input).normalized() * turning_speed
	var steering_error: Vector3 = desired_angular_velocity - world_angular_velocity
	var steering_torque: Vector3 = global_basis * steering_error * steering_responsiveness
	apply_torque(steering_torque)
	
	if anti_roll:
		var banking_torque_direction: Vector3 = basis.z
		var banking_angle: float = -yaw_input * max_banking_angle
		var banking_error: float = deg_to_rad(banking_angle) - rotation.z
		var banking_torque: Vector3 = banking_error * banking_torque_direction * 70000
		apply_torque(banking_torque)
	
