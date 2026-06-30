from p4p.client.thread import Context
from p4p import Value, Type
ctxt = Context("pva")
PREFIX = "ppTest:Path:"

V = Value(Type([
    ('joint_waypoints', 'ad'),
    ('cartesian_waypoints', 'ad'),
    ('resolution', 'd'),
]), {
    'joint_waypoints': [1.0, 2.0, 3.0],
    'resolution': 3.1415
})

result = ctxt.rpc(f"ppTest:Path:Validate", V)

print(f"Valid: {result["valid"]}")
print(f"Description: {result["description"]}")
