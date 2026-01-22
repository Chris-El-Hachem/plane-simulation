extends RigidBody3D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

# https://rnac.com.au/cessna-172/

var currentThrustMagnitude : float = 0
var thrust : Vector3 = Vector3.ZERO
const MAX_THRUST_MAGNITUDE : float = 2150 # N

var frictionCoefficient : float = 0.0319
#const AIR_DENSITY : float = 1.225 # kg/m^3
const DRAG_AREA : float = 0.52 # m^2
var forwardVelocitySquared : float = 0 # m/s
var drag : Vector3 = Vector3.ZERO

const WING_AREA : float = 16.17 # m^2
var liftCoefficient = 1.6
var lift: Vector3 = Vector3.ZERO

func rho(h: float) -> float:
	# https://physics.stackexchange.com/questions/299907/air-density-as-a-function-of-altitude-only
	const T_0 = 288.16 # K
	const alpha = 0.0065 # K/m
	var T = T_0 - alpha * h # K
	const rho_0 = 1.225 # kg/m^2
	const n = 5.2561
	return rho_0 * (T/T_0) ** (n-1)
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if Input.is_action_pressed("thrust"):
		currentThrustMagnitude = MAX_THRUST_MAGNITUDE
	else:
		currentThrustMagnitude = 0
	
	forwardVelocitySquared = linear_velocity.dot(Vector3.FORWARD)**2
	thrust = currentThrustMagnitude * Vector3.FORWARD
	drag = 0.5 * rho(position.y) * frictionCoefficient * DRAG_AREA * forwardVelocitySquared * Vector3.BACK
	lift = 0.5 * rho(position.y) * liftCoefficient * WING_AREA * forwardVelocitySquared * Vector3.UP
	print(linear_velocity)
	print(thrust)
	print(drag)
	print(lift)
	apply_central_force(thrust + drag + lift)
	
	#print(constant_force)
