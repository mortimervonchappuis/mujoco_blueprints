"""Comprehensive tests for World.from_xml_string round-trip."""
import sys
import traceback
import numpy as np
import blueprints as blue
from blueprints.utils.geometry import TAU

PASS = 0
FAIL = 0

def test(name, func):
	global PASS, FAIL
	try:
		func()
		PASS += 1
		print(f"  PASS: {name}")
	except Exception as e:
		FAIL += 1
		print(f"  FAIL: {name}")
		traceback.print_exc()
		print()


def xml_roundtrip(world):
	"""Build world, export XML, reconstruct, export again, compare."""
	xml1 = world.to_xml_string()
	world2 = blue.World.from_xml_string(xml1)
	xml2 = world2.to_xml_string()
	return xml1, xml2, world2


def assert_xml_equal(xml1, xml2, msg=""):
	if xml1 != xml2:
		lines1 = xml1.split('>')
		lines2 = xml2.split('>')
		for i, (a, b) in enumerate(zip(lines1, lines2)):
			if a != b:
				raise AssertionError(f"XML mismatch at element ~{i}: {msg}\n  GOT:      {b}>\n  EXPECTED: {a}>")
		if len(lines1) != len(lines2):
			raise AssertionError(f"XML length mismatch: {len(lines1)} vs {len(lines2)} elements. {msg}")
		raise AssertionError(f"XML mismatch (unknown location). {msg}")


# ===========================================================================
# 1. Basic World parameters
# ===========================================================================
def test_world_params():
	w = blue.World(name='test_world', timestep=0.005, cone='elliptic',
			   viscosity=0.1, gravity=False, contact=False)
	xml1, xml2, w2 = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2, "world params")

test("world params (timestep, cone, viscosity, gravity, contact)", test_world_params)


# ===========================================================================
# 2. Autolimits
# ===========================================================================
def test_autolimits_true():
	w = blue.World(name='autolim', autolimits=True)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_autolimits_false():
	w = blue.World(name='autolim_f', autolimits=False)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("autolimits=True", test_autolimits_true)
test("autolimits=False", test_autolimits_false)


# ===========================================================================
# 3. Single body with each geom type
# ===========================================================================
def test_sphere_geom():
	w = blue.World(name='sphere')
	b = blue.Body(name='b', pos=[1, 2, 3])
	b.attach(blue.geoms.Sphere(size=0.5, color='red'))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_box_geom():
	w = blue.World(name='box')
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Box(size=[0.5, 0.3, 0.2], color='blue'))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_capsule_geom():
	w = blue.World(name='cap')
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Capsule(size=[0.1, 0.5]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_cylinder_geom():
	w = blue.World(name='cyl')
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Cylinder(size=[0.2, 0.4]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_ellipsoid_geom():
	w = blue.World(name='ell')
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Ellipsoid(size=[0.3, 0.2, 0.1]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_plane_geom():
	w = blue.World(name='plane')
	w.attach(blue.geoms.Plane(size=[10, 10, 0.1]))
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("sphere geom", test_sphere_geom)
test("box geom", test_box_geom)
test("capsule geom", test_capsule_geom)
test("cylinder geom", test_cylinder_geom)
test("ellipsoid geom", test_ellipsoid_geom)
test("plane geom", test_plane_geom)


# ===========================================================================
# 4. Multiple geoms on one body
# ===========================================================================
def test_multi_geom():
	w = blue.World(name='multi_geom')
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.3, color='red'))
	b.attach(blue.geoms.Box(size=[0.1, 0.1, 0.5], color='green'))
	b.attach(blue.geoms.Capsule(size=[0.05, 0.3]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("multiple geoms on one body", test_multi_geom)


# ===========================================================================
# 5. Nested body hierarchy
# ===========================================================================
def test_nested_bodies():
	w = blue.World(name='nested')
	root = blue.Body(name='root', pos=[0, 0, 0])
	child1 = blue.Body(name='child1', pos=[1, 0, 0])
	child2 = blue.Body(name='child2', pos=[0, 1, 0])
	grandchild = blue.Body(name='grandchild', pos=[0, 0, 1])
	child1.attach(grandchild)
	root.attach(child1, child2)
	root.attach(blue.geoms.Sphere(size=0.1))
	child1.attach(blue.geoms.Box(size=[0.1, 0.1, 0.1]))
	grandchild.attach(blue.geoms.Capsule(size=[0.05, 0.2]))
	w.attach(root)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("nested body hierarchy (3 levels)", test_nested_bodies)


# ===========================================================================
# 6. Multiple top-level bodies
# ===========================================================================
def test_multiple_toplevel():
	w = blue.World(name='multi_top')
	b1 = blue.Body(name='one', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b2 = blue.Body(name='two', pos=[2, 0, 1], geoms=blue.geoms.Box(size=[0.2, 0.2, 0.2]))
	b3 = blue.Body(name='three', pos=[4, 0, 1], geoms=blue.geoms.Capsule(size=[0.1, 0.3]))
	w.attach(b1, b2, b3)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("multiple top-level bodies", test_multiple_toplevel)


# ===========================================================================
# 7. Joint types
# ===========================================================================
def test_hinge_joint():
	w = blue.World(name='hinge')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.joints.Hinge(axis=[0, 0, 1]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_slide_joint():
	w = blue.World(name='slide')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.joints.Slide(axis=[1, 0, 0]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_free_joint():
	w = blue.World(name='free')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.joints.Free())
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_ball_joint():
	w = blue.World(name='ball')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.joints.Ball())
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("hinge joint", test_hinge_joint)
test("slide joint", test_slide_joint)
test("free joint", test_free_joint)
test("ball joint", test_ball_joint)


# ===========================================================================
# 8. Actuator on joint (Motor)
# ===========================================================================
def test_motor_on_joint():
	w = blue.World(name='motor')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	a = blue.actuators.Motor(gear=[100])
	j.attach(a)
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("motor actuator on joint", test_motor_on_joint)


# ===========================================================================
# 9. General actuator on site
# ===========================================================================
def test_general_on_site():
	w = blue.World(name='gen_site')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	s = blue.sites.Sphere(size=0.05)
	a = blue.actuators.General()
	s.attach(a)
	b.attach(s)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("general actuator on site", test_general_on_site)


# ===========================================================================
# 10. Multiple actuators
# ===========================================================================
def test_multiple_actuators():
	w = blue.World(name='multi_act')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j1 = blue.joints.Hinge(name='j1', axis=[1, 0, 0])
	j2 = blue.joints.Hinge(name='j2', axis=[0, 1, 0])
	a1 = blue.actuators.Motor(gear=[50])
	a2 = blue.actuators.Motor(gear=[75])
	j1.attach(a1)
	j2.attach(a2)
	b.attach(j1, j2)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("multiple actuators on different joints", test_multiple_actuators)


# ===========================================================================
# 11. Sensors: joint sensors
# ===========================================================================
def test_joint_sensors():
	w = blue.World(name='j_sens')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	j.attach(blue.sensors.JointPos())
	j.attach(blue.sensors.JointVel())
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("joint sensors (JointPos, JointVel)", test_joint_sensors)


# ===========================================================================
# 12. Sensors: site sensors
# ===========================================================================
def test_site_sensors():
	w = blue.World(name='s_sens')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	s = blue.sites.Sphere(size=0.05)
	s.attach(blue.sensors.Touch())
	s.attach(blue.sensors.Force())
	b.attach(s)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("site sensors (Touch, Force)", test_site_sensors)


# ===========================================================================
# 13. Camera (no target)
# ===========================================================================
def test_camera_simple():
	w = blue.World(name='cam')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.Camera())
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("camera without target", test_camera_simple)


# ===========================================================================
# 14. Camera with target (use copy=False to keep references)
# ===========================================================================
def test_camera_target():
	w = blue.World(name='cam_tgt')
	b1 = blue.Body(name='viewer', pos=[0, 0, 2], geoms=blue.geoms.Sphere(size=0.1))
	b2 = blue.Body(name='target_body', pos=[2, 0, 0], geoms=blue.geoms.Box(size=[0.2, 0.2, 0.2]))
	cam = blue.Camera(mode='targetbody')
	b1.attach(cam, copy=False)
	w.attach(b1, b2, copy=False)
	cam.target = b2
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("camera with target", test_camera_target)


# ===========================================================================
# 15. Light (simple)
# ===========================================================================
def test_light_simple():
	w = blue.World(name='light')
	b = blue.Body(name='b', pos=[0, 0, 2], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.Light())
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("simple light", test_light_simple)


# ===========================================================================
# 16. Light with target (use copy=False)
# ===========================================================================
def test_light_target():
	w = blue.World(name='light_tgt')
	b1 = blue.Body(name='lamp', pos=[0, 0, 3], geoms=blue.geoms.Sphere(size=0.1))
	b2 = blue.Body(name='lit_obj', pos=[2, 0, 0], geoms=blue.geoms.Box(size=[0.3, 0.3, 0.3]))
	lt = blue.Light(mode='targetbody')
	b1.attach(lt, copy=False)
	w.attach(b1, b2, copy=False)
	lt.target = b2
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("light with target", test_light_target)


# ===========================================================================
# 17. Material with texture (Plane texture with builtin checker)
# ===========================================================================
def test_material_plane_texture():
	w = blue.World(name='mat_tex')
	tex = blue.texture.Plane(builtin='checker', color_1=[1, 0, 0], color_2=[0, 0, 1])
	mat = blue.Material(texture=tex)
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.5, material=mat))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("material with plane texture", test_material_plane_texture)


# ===========================================================================
# 18. Material without texture
# ===========================================================================
def test_material_no_texture():
	w = blue.World(name='mat_no_tex')
	mat = blue.Material(color=[0.8, 0.2, 0.1, 1.0], specular=0.5, shininess=0.3)
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Box(size=[0.3, 0.3, 0.3], material=mat))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("material without texture", test_material_no_texture)


# ===========================================================================
# 19. Material properties (emission, reflectance, etc.)
# ===========================================================================
def test_material_properties():
	w = blue.World(name='mat_props')
	tex = blue.texture.Plane(builtin='checker', color_1=[1, 1, 1], color_2=[0.5, 0.5, 0.5])
	mat = blue.Material(texture=tex, emission=0.3, specular=0.8,
				shininess=0.5, reflectance=0.2)
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.4, material=mat))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("material with all properties", test_material_properties)


# ===========================================================================
# 20. Multiple materials on different bodies
# ===========================================================================
def test_multiple_materials():
	w = blue.World(name='multi_mat')
	tex1 = blue.texture.Plane(builtin='checker', color_1=[1, 0, 0], color_2=[1, 1, 1])
	tex2 = blue.texture.Plane(builtin='gradient', color_1=[0, 0, 1], color_2=[1, 1, 0])
	mat1 = blue.Material(texture=tex1)
	mat2 = blue.Material(texture=tex2)
	b1 = blue.Body(name='b1', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.3, material=mat1))
	b2 = blue.Body(name='b2', pos=[2, 0, 1], geoms=blue.geoms.Box(size=[0.2, 0.2, 0.2], material=mat2))
	w.attach(b1, b2)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("multiple materials on different bodies", test_multiple_materials)


# ===========================================================================
# 21. Skybox texture on world
# ===========================================================================
def test_skybox_texture():
	w = blue.World(name='skybox')
	sky = blue.texture.Skybox(builtin='gradient', color_1=[0.5, 0.7, 1.0], color_2=[1, 1, 1])
	w.texture = sky
	# Need at least one body with a material for skybox to render
	tex = blue.texture.Plane(builtin='checker', color_1=[0, 0.5, 0], color_2=[0.4, 0.2, 0])
	mat = blue.Material(texture=tex)
	w.attach(blue.geoms.Plane(size=[10, 10, 0.1], material=mat))
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("skybox texture on world", test_skybox_texture)


# ===========================================================================
# 22. Geom with color (rgba)
# ===========================================================================
def test_geom_color():
	w = blue.World(name='geom_col')
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.3, color=[0.5, 0.2, 0.8, 0.9]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom with custom rgba color", test_geom_color)


# ===========================================================================
# 23. Body with rotation (euler angles)
# ===========================================================================
def test_body_rotation():
	w = blue.World(name='rot')
	b = blue.Body(name='b', pos=[0, 0, 1], alpha=0.5, beta=0.3, gamma=0.1)
	b.attach(blue.geoms.Box(size=[0.3, 0.2, 0.1]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("body with euler angle rotation", test_body_rotation)


# ===========================================================================
# 24. Site types
# ===========================================================================
def test_site_sphere():
	w = blue.World(name='site_s')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.sites.Sphere(size=0.05))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

def test_site_box():
	w = blue.World(name='site_b')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.sites.Box(size=[0.05, 0.05, 0.05]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("site sphere", test_site_sphere)
test("site box", test_site_box)


# ===========================================================================
# 25. Agent with sensors, actuators, cameras
# ===========================================================================
def test_agent_full():
	w = blue.World(name='agent_full')
	agent = blue.Agent(name='ant', pos=[0, 0, 0.5])
	agent.attach(blue.geoms.Sphere(size=0.3))
	leg = blue.Body(name='leg', pos=[0.3, 0, 0])
	leg.attach(blue.geoms.Capsule(size=[0.05, 0.2]))
	j = blue.joints.Hinge(axis=[0, 1, 0])
	a = blue.actuators.Motor(gear=[50])
	j.attach(a)
	leg.attach(j)
	s = blue.sites.Sphere(size=0.02)
	s.attach(blue.sensors.Touch())
	leg.attach(s)
	agent.attach(blue.Camera())
	agent.attach(leg)
	w.attach(agent)
	xml1, xml2, w2 = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)
	agents = [t for t in w2.all if isinstance(t, blue.Agent)]
	assert len(agents) == 1, f"Expected 1 Agent, got {len(agents)}"

test("agent with full hierarchy", test_agent_full)


# ===========================================================================
# 26. Agent detection (AGENT: prefix in name)
# ===========================================================================
def test_agent_name():
	w = blue.World(name='agent_name')
	agent = blue.Agent(name='robot', pos=[0, 0, 1])
	agent.attach(blue.geoms.Sphere(size=0.2))
	w.attach(agent)
	xml1, xml2, w2 = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)
	agents = [t for t in w2.all if isinstance(t, blue.Agent)]
	assert len(agents) == 1, f"Expected 1 Agent, got {len(agents)}"
	assert 'AGENT:' in agents[0].name

test("agent name detection and reconstruction", test_agent_name)


# ===========================================================================
# 27. Sensor on actuator
# ===========================================================================
def test_actuator_sensor():
	w = blue.World(name='act_sens')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	a = blue.actuators.Motor(gear=[100])
	a.attach(blue.sensors.ActuatorPos())
	j.attach(a)
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("sensor on actuator", test_actuator_sensor)


# ===========================================================================
# 28. Complex model
# ===========================================================================
def test_complex_model():
	w = blue.World(name='complex', timestep=0.002)
	tex = blue.texture.Plane(builtin='checker', color_1=[0, 0.5, 0], color_2=[0.4, 0.2, 0])
	mat = blue.Material(texture=tex)
	ground = blue.geoms.Plane(size=[5, 5, 0.1], material=mat)
	w.attach(ground)

	torso = blue.Body(name='torso', pos=[0, 0, 1])
	torso.attach(blue.geoms.Sphere(size=0.3, color='blue'))
	torso.attach(blue.Camera())
	torso.attach(blue.Light())

	for i in range(2):
		limb = blue.Body(name='limb', pos=[0.3 * (1 if i == 0 else -1), 0, 0])
		limb.attach(blue.geoms.Capsule(size=[0.05, 0.2]))
		j = blue.joints.Hinge(axis=[0, 1, 0])
		a = blue.actuators.Motor(gear=[50])
		j.attach(a)
		limb.attach(j)
		s = blue.sites.Sphere(size=0.02)
		s.attach(blue.sensors.Touch())
		limb.attach(s)
		torso.attach(limb)

	w.attach(torso)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("complex model (multi-body, joints, actuators, sensors, material)", test_complex_model)


# ===========================================================================
# 29. Double round-trip
# ===========================================================================
def test_double_roundtrip():
	w = blue.World(name='double_rt', timestep=0.003)
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.3, color='red'))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	a = blue.actuators.Motor(gear=[30])
	j.attach(a)
	b.attach(j)
	w.attach(b)

	xml1 = w.to_xml_string()
	w2 = blue.World.from_xml_string(xml1)
	xml2 = w2.to_xml_string()
	w3 = blue.World.from_xml_string(xml2)
	xml3 = w3.to_xml_string()
	assert_xml_equal(xml1, xml2, "first round-trip")
	assert_xml_equal(xml2, xml3, "second round-trip")

test("double round-trip", test_double_roundtrip)


# ===========================================================================
# 30. Joint with limits
# ===========================================================================
def test_joint_limits():
	w = blue.World(name='jlim', autolimits=True)
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	j.range = [-1.0, 1.0]
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("joint with range limits", test_joint_limits)


# ===========================================================================
# 31. Geom with friction
# ===========================================================================
def test_geom_friction():
	w = blue.World(name='fric')
	b = blue.Body(name='b', pos=[0, 0, 1])
	g = blue.geoms.Box(size=[0.3, 0.3, 0.3])
	g.friction = [1.0, 0.005, 0.0001]
	b.attach(g)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom with friction", test_geom_friction)


# ===========================================================================
# 32. Body with quaternion orientation
# ===========================================================================
def test_body_quat():
	w = blue.World(name='quat_test')
	b = blue.Body(name='b', pos=[1, 2, 3], quat=[0.7071068, 0.7071068, 0, 0])
	b.attach(blue.geoms.Box(size=[0.2, 0.2, 0.2]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("body with quaternion orientation", test_body_quat)


# ===========================================================================
# 33. World-level geom (directly attached to world, no body)
# ===========================================================================
def test_world_level_geom():
	w = blue.World(name='wlg')
	w.attach(blue.geoms.Plane(size=[10, 10, 0.1]))
	w.attach(blue.geoms.Sphere(size=0.5, pos=[0, 0, 0.5]))
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("world-level geoms (no body)", test_world_level_geom)


# ===========================================================================
# 34. World-level light and camera
# ===========================================================================
def test_world_level_light_camera():
	w = blue.World(name='wlc')
	w.attach(blue.Light(pos=[0, 0, 5]))
	w.attach(blue.Camera(pos=[0, -3, 3]))
	w.attach(blue.geoms.Plane(size=[5, 5, 0.1]))
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("world-level light and camera", test_world_level_light_camera)


# ===========================================================================
# 35. Geom density
# ===========================================================================
def test_geom_density():
	w = blue.World(name='dens')
	b = blue.Body(name='b', pos=[0, 0, 1])
	g = blue.geoms.Sphere(size=0.3)
	g.density = 500.0
	b.attach(g)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom density", test_geom_density)


# ===========================================================================
# 36. Geom with contype/conaffinity
# ===========================================================================
def test_geom_contact_params():
	w = blue.World(name='contact')
	b = blue.Body(name='b', pos=[0, 0, 1])
	g = blue.geoms.Sphere(size=0.3)
	g.contype = 1
	g.conaffinity = 0
	b.attach(g)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom contype/conaffinity", test_geom_contact_params)


# ===========================================================================
# 37. Joint with damping
# ===========================================================================
def test_joint_damping():
	w = blue.World(name='damp')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	j.damping = 0.5
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("joint with damping", test_joint_damping)


# ===========================================================================
# 38. Named things
# ===========================================================================
def test_named_things():
	w = blue.World(name='named')
	b = blue.Body(name='torso', pos=[0, 0, 1])
	g = blue.geoms.Sphere(name='torso_geom', size=0.3)
	j = blue.joints.Hinge(name='hip', axis=[0, 0, 1])
	s = blue.sites.Sphere(name='contact_site', size=0.05)
	c = blue.Camera(name='eye')
	l = blue.Light(name='headlight')
	b.attach(g, j, s, c, l)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("named things preserve names", test_named_things)


# ===========================================================================
# 39. HField geom
# ===========================================================================
def test_hfield_geom():
	w = blue.World(name='hfield')
	data = np.random.rand(10, 10).astype(np.float32)
	hf = blue.geoms.HField(terrain=data, size=[5, 5, 1, 0.1])
	w.attach(hf)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("HField geom", test_hfield_geom)


# ===========================================================================
# 40. Sensor + actuator on same joint
# ===========================================================================
def test_sensor_actuator_same_joint():
	w = blue.World(name='sa_joint')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	j.attach(blue.sensors.JointPos())
	j.attach(blue.actuators.Motor(gear=[50]))
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("sensor and actuator on same joint", test_sensor_actuator_same_joint)


# ===========================================================================
# 41. Geom group attribute
# ===========================================================================
def test_geom_group():
	w = blue.World(name='ggroup')
	b = blue.Body(name='b', pos=[0, 0, 1])
	g = blue.geoms.Sphere(size=0.3)
	g.group = 1
	b.attach(g)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom group attribute", test_geom_group)


# ===========================================================================
# 42. Ant-like model (realistic multi-leg agent)
# ===========================================================================
def test_ant_model():
	w = blue.World(name='ant', timestep=0.01)
	w.attach(blue.geoms.Plane(size=[10, 10, 0.1]))

	agent = blue.Agent(name='ant', pos=[0, 0, 0.75])
	torso_geom = blue.geoms.Sphere(size=0.25)
	agent.attach(torso_geom)
	agent.attach(blue.Camera(name='ant_cam'))

	angles = [TAU/8, 3*TAU/8, 5*TAU/8, 7*TAU/8]
	for i, angle in enumerate(angles):
		hip = blue.Body(name=f'hip_{i}', pos=[0.25 * np.cos(angle), 0.25 * np.sin(angle), 0])
		hip.attach(blue.geoms.Capsule(size=[0.04, 0.15]))
		hip_j = blue.joints.Hinge(name=f'hip_j_{i}', axis=[0, 0, 1])
		hip_j.attach(blue.actuators.Motor(gear=[50]))
		hip.attach(hip_j)

		knee = blue.Body(name=f'knee_{i}', pos=[0.15, 0, 0])
		knee.attach(blue.geoms.Capsule(size=[0.04, 0.15]))
		knee_j = blue.joints.Hinge(name=f'knee_j_{i}', axis=[0, 1, 0])
		knee_j.attach(blue.actuators.Motor(gear=[50]))
		knee.attach(knee_j)

		foot_site = blue.sites.Sphere(name=f'foot_{i}', size=0.02)
		foot_site.attach(blue.sensors.Touch())
		knee.attach(foot_site)

		hip.attach(knee)
		agent.attach(hip)

	w.attach(agent)
	xml1, xml2, w2 = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2, "ant model")

	agents = [t for t in w2.all if isinstance(t, blue.Agent)]
	assert len(agents) == 1, f"Expected 1 Agent, got {len(agents)}"

test("ant-like multi-leg agent model", test_ant_model)


# ===========================================================================
# 43. Joint with stiffness, armature, springref
# ===========================================================================
def test_joint_scalar_attrs():
	w = blue.World(name='jscalar')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1], damping=1.0, stiffness=5.0,
				  armature=0.1, springref=0.5)
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("joint scalar attrs (damping, stiffness, armature, springref)", test_joint_scalar_attrs)


# ===========================================================================
# 44. Material with texrepeat
# ===========================================================================
def test_material_texrepeat():
	w = blue.World(name='texrep')
	tex = blue.texture.Plane(builtin='checker', color_1=[1, 0, 0], color_2=[0, 0, 1])
	mat = blue.Material(texture=tex, texrepeat=[3, 3])
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.5, material=mat))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("material with texrepeat", test_material_texrepeat)


# ===========================================================================
# 45. Empty world (no bodies)
# ===========================================================================
def test_empty_world():
	w = blue.World(name='empty')
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("empty world", test_empty_world)


# ===========================================================================
# 46. Body with no geom (structural body)
# ===========================================================================
def test_structural_body():
	w = blue.World(name='struct')
	root = blue.Body(name='root', pos=[0, 0, 0])
	child = blue.Body(name='child', pos=[1, 0, 0])
	child.attach(blue.geoms.Sphere(size=0.1))
	root.attach(child)
	w.attach(root)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("structural body (no geom)", test_structural_body)


# ===========================================================================
# 47. Joint with springdamper (multi-element array)
# ===========================================================================
def test_joint_springdamper():
	w = blue.World(name='sd')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1], springdamper=[10.0, 1.0])
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("joint with springdamper (multi-element array)", test_joint_springdamper)


# ===========================================================================
# 48. Double round-trip with material
# ===========================================================================
def test_double_roundtrip_material():
	w = blue.World(name='drt_mat')
	tex = blue.texture.Plane(builtin='checker', color_1=[1, 0, 0], color_2=[0, 0, 1])
	mat = blue.Material(texture=tex, specular=0.8)
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.3, material=mat))
	w.attach(b)

	xml1 = w.to_xml_string()
	w2 = blue.World.from_xml_string(xml1)
	xml2 = w2.to_xml_string()
	w3 = blue.World.from_xml_string(xml2)
	xml3 = w3.to_xml_string()
	assert_xml_equal(xml1, xml2, "first round-trip")
	assert_xml_equal(xml2, xml3, "second round-trip")

test("double round-trip with material", test_double_roundtrip_material)


# ===========================================================================
# 49. Geom with position offset
# ===========================================================================
def test_geom_pos_offset():
	w = blue.World(name='gpos')
	b = blue.Body(name='b', pos=[0, 0, 1])
	g = blue.geoms.Sphere(size=0.2, pos=[0.5, 0, 0])
	b.attach(g)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom with position offset", test_geom_pos_offset)


# ===========================================================================
# 50. Multiple cameras on one body
# ===========================================================================
def test_multi_camera():
	w = blue.World(name='mcam')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.Camera(name='front'))
	b.attach(blue.Camera(name='top', pos=[0, 0, 1]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("multiple cameras on one body", test_multi_camera)


# ===========================================================================
# 51. Deeply nested bodies (5 levels)
# ===========================================================================
def test_deep_nesting():
	w = blue.World(name='deep')
	parent = blue.Body(name='L0', pos=[0, 0, 1])
	parent.attach(blue.geoms.Sphere(size=0.1))
	current = parent
	for i in range(1, 5):
		child = blue.Body(name=f'L{i}', pos=[0.3, 0, 0])
		child.attach(blue.geoms.Sphere(size=0.05))
		current.attach(child)
		current = child
	w.attach(parent)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("deeply nested bodies (5 levels)", test_deep_nesting)


# ===========================================================================
# 52. Multiple sensors on same joint
# ===========================================================================
def test_multi_sensors_one_joint():
	w = blue.World(name='ms_joint')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	j.attach(blue.sensors.JointPos())
	j.attach(blue.sensors.JointVel())
	j.attach(blue.actuators.Motor(gear=[30]))
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("multiple sensors + actuator on same joint", test_multi_sensors_one_joint)


# ===========================================================================
# 53. Two agents in one world
# ===========================================================================
def test_two_agents():
	w = blue.World(name='two_agents')
	for name, x in [('agent_a', 0), ('agent_b', 3)]:
		agent = blue.Agent(name=name, pos=[x, 0, 0.5])
		agent.attach(blue.geoms.Sphere(size=0.2))
		leg = blue.Body(name=f'{name}_leg', pos=[0.2, 0, 0])
		leg.attach(blue.geoms.Capsule(size=[0.04, 0.15]))
		j = blue.joints.Hinge(axis=[0, 1, 0])
		j.attach(blue.actuators.Motor(gear=[40]))
		leg.attach(j)
		agent.attach(leg)
		w.attach(agent)
	xml1, xml2, w2 = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)
	agents = [t for t in w2.all if isinstance(t, blue.Agent)]
	assert len(agents) == 2, f"Expected 2 Agents, got {len(agents)}"

test("two agents in one world", test_two_agents)


# ===========================================================================
# 54. Shared texture between materials
# ===========================================================================
def test_shared_texture():
	w = blue.World(name='shared_tex')
	tex = blue.texture.Plane(builtin='checker', color_1=[1, 0, 0], color_2=[0, 0, 1])
	mat1 = blue.Material(texture=tex, specular=0.3)
	mat2 = blue.Material(texture=tex, specular=0.8)
	b1 = blue.Body(name='b1', pos=[0, 0, 1])
	b1.attach(blue.geoms.Sphere(size=0.3, material=mat1))
	b2 = blue.Body(name='b2', pos=[2, 0, 1])
	b2.attach(blue.geoms.Sphere(size=0.3, material=mat2))
	w.attach(b1, b2)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("shared texture between two materials", test_shared_texture)


# ===========================================================================
# 55. Body with multiple joints
# ===========================================================================
def test_multi_joints():
	w = blue.World(name='mj')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	b.attach(blue.joints.Hinge(name='j_x', axis=[1, 0, 0]))
	b.attach(blue.joints.Hinge(name='j_y', axis=[0, 1, 0]))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("body with multiple joints", test_multi_joints)


# ===========================================================================
# 56. Triple round-trip
# ===========================================================================
def test_triple_roundtrip():
	w = blue.World(name='triple_rt', timestep=0.004)
	tex = blue.texture.Plane(builtin='checker', color_1=[1, 0, 0], color_2=[0, 1, 0])
	mat = blue.Material(texture=tex)
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Sphere(size=0.3, material=mat))
	j = blue.joints.Hinge(axis=[0, 0, 1], damping=0.5)
	j.attach(blue.actuators.Motor(gear=[30]))
	j.attach(blue.sensors.JointPos())
	b.attach(j)
	w.attach(b)

	xml1 = w.to_xml_string()
	w2 = blue.World.from_xml_string(xml1)
	xml2 = w2.to_xml_string()
	w3 = blue.World.from_xml_string(xml2)
	xml3 = w3.to_xml_string()
	w4 = blue.World.from_xml_string(xml3)
	xml4 = w4.to_xml_string()
	assert_xml_equal(xml1, xml2, "1st round-trip")
	assert_xml_equal(xml2, xml3, "2nd round-trip")
	assert_xml_equal(xml3, xml4, "3rd round-trip")

test("triple round-trip with material+actuator+sensor", test_triple_roundtrip)


# ===========================================================================
# 57. Build and simulate after reconstruction
# ===========================================================================
def test_simulate_after_reconstruct():
	w = blue.World(name='sim_test', timestep=0.01)
	w.attach(blue.geoms.Plane(size=[5, 5, 0.1]))
	b = blue.Body(name='ball', pos=[0, 0, 2])
	b.attach(blue.geoms.Sphere(size=0.1))
	b.attach(blue.joints.Free())
	w.attach(b)

	xml1 = w.to_xml_string()
	w2 = blue.World.from_xml_string(xml1)
	w2.build()
	# Step 100 times — should not crash
	w2.step(n_steps=100)
	# Ball should have fallen
	z = w2.data.qpos[2]
	assert z < 2.0, f"Ball should have fallen, z={z}"
	w2.unbuild()
	# Verify XML still matches after build/simulate/unbuild
	xml2 = w2.to_xml_string()
	assert_xml_equal(xml1, xml2, "XML should match after simulate+unbuild")

test("build and simulate after reconstruction", test_simulate_after_reconstruct)


# ===========================================================================
# 58. Slide joint with actuator
# ===========================================================================
def test_slide_actuator():
	w = blue.World(name='slide_act')
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.2))
	j = blue.joints.Slide(axis=[0, 0, 1])
	j.attach(blue.actuators.Motor(gear=[100]))
	b.attach(j)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("slide joint with motor actuator", test_slide_actuator)


# ===========================================================================
# 59. Geom with mass instead of density
# ===========================================================================
def test_geom_mass():
	w = blue.World(name='gmass')
	b = blue.Body(name='b', pos=[0, 0, 1])
	g = blue.geoms.Box(size=[0.2, 0.2, 0.2])
	g.mass = 5.0
	b.attach(g)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom with mass", test_geom_mass)


# ===========================================================================
# 60. Multiple bodies with joints, actuators, and sensors
# ===========================================================================
def test_multi_body_chain():
	w = blue.World(name='chain')
	prev = blue.Body(name='link_0', pos=[0, 0, 2])
	prev.attach(blue.geoms.Sphere(size=0.1))
	w.attach(prev)
	for i in range(1, 4):
		link = blue.Body(name=f'link_{i}', pos=[0.3, 0, 0])
		link.attach(blue.geoms.Capsule(size=[0.03, 0.15]))
		j = blue.joints.Hinge(axis=[0, 1, 0], damping=0.3)
		j.attach(blue.actuators.Motor(gear=[20]))
		j.attach(blue.sensors.JointPos())
		link.attach(j)
		prev.attach(link)
		prev = link
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("multi-body chain with joints+actuators+sensors", test_multi_body_chain)


# ===========================================================================
# 61. World-level site with sensor
# ===========================================================================
def test_world_level_site():
	w = blue.World(name='wsite')
	s = blue.sites.Sphere(size=0.05, pos=[1, 0, 0])
	s.attach(blue.sensors.Touch())
	w.attach(s)
	w.attach(blue.geoms.Plane(size=[5, 5, 0.1]))
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("world-level site with sensor", test_world_level_site)


# ===========================================================================
# 62. Cube texture (Box texture type)
# ===========================================================================
def test_cube_texture():
	w = blue.World(name='cube_tex')
	tex = blue.texture.Box(builtin='checker', color_1=[0, 1, 0], color_2=[1, 1, 0])
	mat = blue.Material(texture=tex)
	b = blue.Body(name='b', pos=[0, 0, 1])
	b.attach(blue.geoms.Box(size=[0.3, 0.3, 0.3], material=mat))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("cube (Box) texture type", test_cube_texture)


# ===========================================================================
# 63. Geom with condim
# ===========================================================================
def test_geom_condim():
	w = blue.World(name='condim')
	b = blue.Body(name='b', pos=[0, 0, 1])
	g = blue.geoms.Sphere(size=0.3)
	g.condim = 4
	b.attach(g)
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("geom with condim", test_geom_condim)


# ===========================================================================
# 64. Skybox without other materials (standalone)
# ===========================================================================
def test_skybox_standalone():
	w = blue.World(name='sky_only')
	sky = blue.texture.Skybox(builtin='gradient', color_1=[0.3, 0.5, 0.9], color_2=[1, 1, 1])
	w.texture = sky
	b = blue.Body(name='b', pos=[0, 0, 1], geoms=blue.geoms.Sphere(size=0.3))
	w.attach(b)
	xml1, xml2, _ = xml_roundtrip(w)
	assert_xml_equal(xml1, xml2)

test("skybox texture standalone (no materials)", test_skybox_standalone)


# ===========================================================================
# 65. Agent with camera and observation
# ===========================================================================
def test_agent_observation():
	w = blue.World(name='agent_obs')
	agent = blue.Agent(name='obs_agent', pos=[0, 0, 1])
	agent.attach(blue.geoms.Sphere(size=0.2))
	j = blue.joints.Hinge(axis=[0, 0, 1])
	j.attach(blue.sensors.JointPos())
	j.attach(blue.actuators.Motor(gear=[50]))
	agent.attach(j)
	agent.attach(blue.Camera(name='obs_cam'))
	w.attach(agent)

	xml1 = w.to_xml_string()
	w2 = blue.World.from_xml_string(xml1)
	xml2 = w2.to_xml_string()
	assert_xml_equal(xml1, xml2)

	# Verify agent can produce observations after reconstruction
	w2.build()
	agents = [t for t in w2.all if isinstance(t, blue.Agent)]
	assert len(agents) == 1
	obs = agents[0].observation
	assert isinstance(obs, dict), f"Expected dict, got {type(obs)}"
	w2.unbuild()

test("agent observation after reconstruction", test_agent_observation)


# ===========================================================================
# Summary
# ===========================================================================
print(f"\n{'='*60}")
print(f"Results: {PASS} passed, {FAIL} failed out of {PASS + FAIL} tests")
print(f"{'='*60}")
sys.exit(1 if FAIL else 0)
