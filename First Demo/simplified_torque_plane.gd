extends RigidBody3D

# Major credit to Vazgriz
# https://www.youtube.com/watch?v=7vAHo2B1zLc
# https://github.com/vazgriz/FlightSim

var wind: Vector3 = Vector3.ZERO

var max_thrust: float = 5000

var min_drag_coefficient: float = 1
var max_drag_coefficient: float = 3

@export var lift_curve: Curve
var base_angle_of_attack: float = 4
var lift_power: float = 30

var angular_drag_coefficient: float = 1

var turning_speed: float = 1 # rad/s
var steering_responsiveness: float = 50000

func _physics_process(delta: float) -> void:
	var thrust_direction: Vector3 = -basis.z
	var thrust_magnitude: float = Input.get_action_strength("thrust") * max_thrust
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
	
	var angular_drag: Vector3 = angular_drag_coefficient * angular_velocity.length_squared() * -angular_velocity.normalized()
	apply_torque(angular_drag)
	
	var yaw_input: float = Input.get_axis("yaw left", "yaw right")
	var pitch_input: float = Input.get_axis("pitch up", "pitch down")
	var roll_input: float = Input.get_axis("roll left", "roll right")
	var desired_angular_velocity: Vector3 = Vector3(-pitch_input, -yaw_input, -roll_input).normalized() * turning_speed 
	var steering_error: Vector3 = desired_angular_velocity - global_basis.inverse() * angular_velocity
	var steering_input: Vector3 = global_basis * steering_error * steering_responsiveness
	apply_torque(steering_input)
