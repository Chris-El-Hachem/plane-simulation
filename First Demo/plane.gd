extends RigidBody3D

var currentThrustMagnitude: float = 0
var thrust: Vector3 = Vector3.ZERO
const MAX_THRUST_MAGNITUDE: float = 9000                    # N

var wind: Vector3 = Vector3.ZERO

@onready var left_wing: AeroSurface = $Surfaces/LeftWing
@onready var right_wing: AeroSurface = $Surfaces/RightWing
@onready var left_tail: AeroSurface = $Surfaces/LeftTail
@onready var right_tail: AeroSurface = $Surfaces/RightTail

func air_density(height: float) -> float:
	# https://physics.stackexchange.com/questions/299907/air-density-as-a-function-of-altitude-only
	const T_0 = 288.16 # K
	const alpha = 0.0065 # K/m
	var T: float = T_0 - alpha * height # K
	const air_density_sea_level = 1.225 # kg/m^2
	const n = 5.2561
	return air_density_sea_level * pow(T/T_0, n-1)

func _ready() -> void:
	#
	#left_wing.set_flap_angle(20)
	#right_wing.set_flap_angle(20)
	#print('flaps set')
	#for i in range(1000):
		#var specific_angle: float = float(i)/10
		#var air_flow: Vector3 = Vector3(0, sin(deg_to_rad(specific_angle)), -cos(deg_to_rad(specific_angle)))
		#var forces: Vector3 = left_wing.calculate_forces(air_flow, 1.225)
		#print(specific_angle)
	pass

func _physics_process(delta: float) -> void:
	var forward: Vector3 = -global_transform.basis.z
	
	# Thrust
	if Input.is_action_pressed("thrust"):
		currentThrustMagnitude = MAX_THRUST_MAGNITUDE
	else:
		currentThrustMagnitude = 0
	thrust = currentThrustMagnitude * forward
	
	if Input.is_action_pressed("right"):
		print("right")
		left_wing.set_flap_angle(1)
		right_wing.set_flap_angle(-1)
	elif Input.is_action_pressed("left"):
		print("left")
		left_wing.set_flap_angle(-1)
		right_wing.set_flap_angle(1)
	else:
		left_wing.set_flap_angle(0)
		right_wing.set_flap_angle(0)
		
	if Input.is_action_pressed("pitch up"):
		print("left")
		left_tail.set_flap_angle(-4)
		right_tail.set_flap_angle(-4)
	elif Input.is_action_pressed("pitch down"):
		print("right")
		left_tail.set_flap_angle(4)
		right_tail.set_flap_angle(4)
	else:
		left_tail.set_flap_angle(0)
		right_tail.set_flap_angle(0)
	
	var sum_of_forces: Vector3 = thrust
	var sum_of_torques: Vector3 = Vector3.ZERO
	
	var world_flow_velocity: Vector3 = -linear_velocity + wind
	
	for surface in $Surfaces.get_children():
		
		var inverse_global_transform = global_transform.inverse()
		var lift_direction = inverse_global_transform.basis * surface.global_transform.basis.y
		var drag_direction = inverse_global_transform.basis * surface.global_transform.basis.z
		var moment_direction = inverse_global_transform.basis * -surface.global_transform.basis.x
		
		var local_flow_velocity: Vector3 = surface.global_transform.inverse().basis * (world_flow_velocity - angular_velocity.cross(surface.position))
		local_flow_velocity.x = 0
		#print(local_flow_velocity)
		
		var forces: Vector3 = surface.calculate_forces(local_flow_velocity, air_density(position.y))
		
		#print(forces)
		var lift: Vector3 = forces.x * lift_direction
		#print(forces.x)
		var drag: Vector3 = forces.y * drag_direction
		var moment: Vector3 = forces.z * moment_direction
		#print("forces" + str(forces))
		
		sum_of_forces += lift + drag
		sum_of_torques += moment + surface.position.cross(lift + drag)
		
	#print(linear_velocity)
	apply_central_force(sum_of_forces)
	apply_torque(sum_of_torques)
	
