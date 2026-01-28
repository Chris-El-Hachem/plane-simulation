extends RigidBody3D

const GRAVITATIONAL_CONSTANT = 9.8
var max_thrust: float = 5000
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass


func _physics_process(delta: float) -> void:
	
	var thrust_magnitude: float = Input.get_action_strength("thrust") * max_thrust
	print(Input.get_action_strength("thrust"))
		#print("thrusted")
	var thrust: Vector3 = thrust_magnitude * Vector3.FORWARD
	apply_central_force(thrust)
	pass
