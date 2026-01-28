@tool
extends Node3D
class_name AeroSurface

# Credit to "Jump Trajectory"/"gasgiant"
# https://github.com/gasgiant/Aircraft-Physics
# https://www.youtube.com/watch?v=p3jDJ9FtTyM

# Inherent wing parameters that affect lift/drag/moment
@export var lift_slope: float = 6.28
@export var skin_friction: float = 0.02
@export var zero_lift_aoa_deg: float = 0.0
@export_range(0, 90) var high_stall_angle_deg: float = 15.0
@export_range(-90, 0) var low_stall_angle_deg: float = -15.0
@export var chord: float = 1.0:
	set(value):
		chord = value
		_update_surface(chord, flap_fraction, span)
@export_range(0.0, 0.4) var flap_fraction: float = 0.0:
	set(value):
		flap_fraction = value
		_update_surface(chord, flap_fraction, span)
@export var span: float = 1.0:
	set(value):
		span = value
		_update_surface(chord, flap_fraction, span)
var aspect_ratio: float = span/chord

# Flap parameters
@export var max_flap_angle_deg: float = 50.0
@export var min_flap_angle_deg: float = -50.0
var _flap_angle_rad: float = 0.0

# Surface display for editor
var surface_mesh_instance: MeshInstance3D = MeshInstance3D.new()
var surface_mesh: BoxMesh = BoxMesh.new()
var control_mesh_instance: MeshInstance3D = MeshInstance3D.new()
var control_mesh: BoxMesh = BoxMesh.new()

func _ready() -> void:
	if Engine.is_editor_hint():
		_display_surface(chord, flap_fraction, span)

func _display_surface(new_chord: float, new_flap_fraction: float, new_span: float) -> void:
	surface_mesh.size = Vector3(new_span, 0.1, new_chord * (1 - new_flap_fraction))
	var surface_material: StandardMaterial3D = StandardMaterial3D.new()
	var surface_color: Color = Color(0.0, 0.8, 1.0, 1.0)
	surface_material.albedo_color = surface_color
	surface_mesh.material = surface_material
	surface_mesh_instance.mesh = surface_mesh
	add_child(surface_mesh_instance)
	surface_mesh_instance.position = Vector3(0, 0, -new_chord * new_flap_fraction / 2)
	
	control_mesh.size = Vector3(new_span, 0.1, new_chord * new_flap_fraction)
	var control_material: StandardMaterial3D = StandardMaterial3D.new()
	var control_color: Color = Color(1.0, 0.7, 0.0, 1.0)
	control_material.albedo_color = control_color
	control_mesh.material = control_material
	control_mesh_instance.mesh = control_mesh
	add_child(control_mesh_instance)
	control_mesh_instance.position = Vector3(0, 0, new_chord * (1 - new_flap_fraction) / 2)

func _update_surface(new_chord: float, new_flap_fraction: float, new_span: float) -> void:
	surface_mesh.size = Vector3(new_span, 0.1, new_chord * (1 - new_flap_fraction))
	surface_mesh_instance.position = Vector3(0, 0, -new_chord * new_flap_fraction / 2)
	
	control_mesh.size = Vector3(new_span, 0.1, new_chord * new_flap_fraction)
	control_mesh_instance.position = Vector3(0, 0, new_chord * (1 - new_flap_fraction) / 2)

func set_flap_angle(angle: float) -> void:
	_flap_angle_rad = clamp(angle, deg_to_rad(min_flap_angle_deg), deg_to_rad(max_flap_angle_deg))

func calculate_forces(local_flow_velocity: Vector3, air_density: float) -> Vector3:
	var forces: Vector3 = Vector3.ZERO
	var corrected_lift_slope: float = lift_slope * aspect_ratio / (aspect_ratio + 2 * (aspect_ratio + 4) / (aspect_ratio + 2))
	
	var theta: float = acos(2 * flap_fraction - 1)
	var flap_effectiveness: float = 1 - (theta - sin(theta)) / PI
	var delta_lift: float = corrected_lift_slope * flap_effectiveness * _flap_effectiveness_correction(_flap_angle_rad) * _flap_angle_rad
	
	var zero_lift_aoa_rad: float = deg_to_rad(zero_lift_aoa_deg)
	var zero_lift_aoa: float = zero_lift_aoa_rad - delta_lift / corrected_lift_slope
	
	var high_stall_angle_rad: float = deg_to_rad(high_stall_angle_deg)
	var low_stall_angle_rad: float = deg_to_rad(low_stall_angle_deg)
	
	var cl_max_high: float = corrected_lift_slope * (high_stall_angle_rad - zero_lift_aoa_rad) + delta_lift * _lift_coefficient_max_fraction(flap_fraction)
	var cl_max_low: float = corrected_lift_slope * (low_stall_angle_rad - zero_lift_aoa_rad) + delta_lift * _lift_coefficient_max_fraction(flap_fraction)
	
	var stall_angle_high: float = zero_lift_aoa + cl_max_high / corrected_lift_slope
	var stall_angle_low: float = zero_lift_aoa + cl_max_low / corrected_lift_slope
	
	var area: float = chord * span
	var dynamic_pressure: float = 0.5 * air_density * local_flow_velocity.length_squared()
	var angle_of_attack: float = atan2(local_flow_velocity.y, local_flow_velocity.z)
	#print(angle_of_attack)
	
	var coefficients: Vector3 = _calculate_coefficients(angle_of_attack, corrected_lift_slope, zero_lift_aoa, stall_angle_high, stall_angle_low)
	forces = coefficients * dynamic_pressure * area
	forces.z *= chord
	return forces

func _calculate_coefficients(angle_of_attack: float, corrected_lift_slope: float, zero_lift_aoa: float, stall_angle_high: float, stall_angle_low: float) -> Vector3:
	var coefficients: Vector3
	
	var padding_angle_high: float = deg_to_rad(lerp(15, 5, (rad_to_deg(_flap_angle_rad) - min_flap_angle_deg) / (max_flap_angle_deg - min_flap_angle_deg)))
	var padding_angle_low: float = deg_to_rad(lerp(15, 5, (-rad_to_deg(_flap_angle_rad) - min_flap_angle_deg) / (max_flap_angle_deg - min_flap_angle_deg)))
	#var padding_angle_high: float = deg_to_rad(lerp(15, 5, (rad_to_deg(_flap_angle_rad) + 50) / (100)))
	#var padding_angle_low: float = deg_to_rad(lerp(15, 5, (-rad_to_deg(_flap_angle_rad) + 50) / (100)))
	var padded_stall_angle_high: float = stall_angle_high + padding_angle_high
	var padded_stall_angle_low: float = stall_angle_low - padding_angle_low
	
	if angle_of_attack < stall_angle_high and angle_of_attack > stall_angle_low:
		coefficients = _calculate_coefficients_at_low_aoa(angle_of_attack, corrected_lift_slope, zero_lift_aoa)
	else: 
		if angle_of_attack > padded_stall_angle_high or angle_of_attack < padded_stall_angle_low:
			coefficients = _calculate_coefficients_at_stall(angle_of_attack, corrected_lift_slope, zero_lift_aoa, stall_angle_high, stall_angle_low)
		else:
			var coefficients_low: Vector3
			var coefficients_stall: Vector3
			var lerp_param: float
			if angle_of_attack > stall_angle_high:
				coefficients_low = _calculate_coefficients_at_low_aoa(stall_angle_high, corrected_lift_slope, zero_lift_aoa)
				coefficients_stall = _calculate_coefficients_at_stall(padded_stall_angle_high, corrected_lift_slope, zero_lift_aoa, stall_angle_high, stall_angle_low)
				lerp_param = (angle_of_attack - stall_angle_high) / (padded_stall_angle_high - stall_angle_high)
			else:
				coefficients_low = _calculate_coefficients_at_low_aoa(stall_angle_low, corrected_lift_slope, zero_lift_aoa)
				coefficients_stall = _calculate_coefficients_at_stall(padded_stall_angle_low, corrected_lift_slope, zero_lift_aoa, stall_angle_high, stall_angle_low)
				lerp_param = (angle_of_attack - stall_angle_low) / (padded_stall_angle_low - stall_angle_low)
			coefficients = coefficients_low.lerp(coefficients_stall, lerp_param)
	return coefficients

func _calculate_coefficients_at_low_aoa(angle_of_attack: float, corrected_lift_slope: float, zero_lift_aoa: float) -> Vector3:
	var lift_coefficient: float = corrected_lift_slope * (angle_of_attack - zero_lift_aoa)
	var induced_angle: float = lift_coefficient / (PI * aspect_ratio)
	var effective_angle: float = angle_of_attack - zero_lift_aoa - induced_angle
	
	var tangential_coefficient: float = skin_friction * cos(effective_angle)
	
	var normal_coefficient: float = (lift_coefficient + sin(effective_angle) * tangential_coefficient) / cos(effective_angle)
	var drag_coefficient: float = normal_coefficient * sin(effective_angle) + tangential_coefficient * cos(effective_angle)
	var moment_coefficient: float = -normal_coefficient * _torque_coefficient_proportion(effective_angle)
	
	return Vector3(lift_coefficient, drag_coefficient, moment_coefficient)

func _calculate_coefficients_at_stall(angle_of_attack: float, corrected_lift_slope: float, zero_lift_aoa: float, stall_angle_high: float, stall_angle_low: float) -> Vector3:
	var lift_coefficient_low_aoa: float
	if angle_of_attack > stall_angle_high:
		lift_coefficient_low_aoa = corrected_lift_slope * (stall_angle_high - zero_lift_aoa)
	else:
		lift_coefficient_low_aoa = corrected_lift_slope * (stall_angle_low - zero_lift_aoa)
	
	var induced_angle: float = lift_coefficient_low_aoa / (PI * aspect_ratio)
	
	var lerp_param: float
	if angle_of_attack > stall_angle_high:
		lerp_param = (PI/2 - clamp(angle_of_attack, -PI/2, PI/2)) / (PI/2 - stall_angle_high)
	else:
		lerp_param = (-PI/2 - clamp(angle_of_attack, -PI/2, PI/2)) / (-PI/2 - stall_angle_low)
	induced_angle = lerp(0.0, induced_angle, lerp_param)
	var effective_angle: float = angle_of_attack - zero_lift_aoa - induced_angle
	
	var normal_coefficient: float = _friction_at_90_degrees(_flap_angle_rad) * sin(effective_angle) * (1 / (0.56 + 0.44 * abs(sin(effective_angle))) - 0.41 * (1 - exp(-17/aspect_ratio)))
	var tangential_coefficient: float = 0.5 * skin_friction * cos(effective_angle)
	
	var lift_coefficient: float = normal_coefficient * cos(effective_angle) - tangential_coefficient * sin(effective_angle)
	var drag_coefficient: float = normal_coefficient * sin(effective_angle) + tangential_coefficient * cos(effective_angle)
	var moment_coefficient: float = -normal_coefficient * _torque_coefficient_proportion(effective_angle)
	
	return Vector3(lift_coefficient, drag_coefficient, moment_coefficient)

func _torque_coefficient_proportion(angle: float) -> float:
	return 0.25 - 0.175 * (1.0 - 2.0 * abs(angle) / PI)

func _friction_at_90_degrees(angle: float) -> float:
	return 1.98 - 0.0426 * angle * angle + 0.21 * angle

func _flap_effectiveness_correction(angle: float) -> float:
	return lerp(0.8, 0.4, (rad_to_deg(abs(angle)) - 10.0)/50.0)

func _lift_coefficient_max_fraction(fraction: float) -> float:
	return clamp(1.0 - 0.5 * (fraction - 0.1) / 0.3, 0.0, 1.0)
