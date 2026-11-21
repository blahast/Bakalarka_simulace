import xml.etree.ElementTree as ET
import numpy as np
import mujoco

def parse_vec(s, default):
    return np.fromstring(s, sep=" ") if s else np.array(default, dtype=float)

def fmt_vec(v):
    return " ".join(f"{x:.8g}" for x in v)

def flatten_frames(parent, f_pos=None, f_quat=None):
    if f_pos is None:
        f_pos = np.zeros(3)
        f_quat = np.array([1.0, 0.0, 0.0, 0.0])

    new_children = []
    for child in list(parent):
        if child.tag == "frame":
            c_pos = parse_vec(child.get("pos"), [0, 0, 0])
            c_quat = parse_vec(child.get("quat"), [1, 0, 0, 0])
            pos_out, quat_out = np.zeros(3), np.zeros(4)
            mujoco.mju_mulPose(pos_out, quat_out, f_pos, f_quat, c_pos, c_quat)
            new_children.extend(flatten_frames(child, pos_out, quat_out))
        else:
            if child.tag in ("body", "geom", "site"):
                c_pos = parse_vec(child.get("pos"), [0, 0, 0])
                c_quat = parse_vec(child.get("quat"), [1, 0, 0, 0])
                pos_out, quat_out = np.zeros(3), np.zeros(4)
                mujoco.mju_mulPose(pos_out, quat_out, f_pos, f_quat, c_pos, c_quat)
                child.set("pos", fmt_vec(pos_out))
                child.set("quat", fmt_vec(quat_out))
            # Rekurzivně zploštit případné <frame> uvnitř <body>
            if child.tag == "body":
                child[:] = flatten_frames(child)
            new_children.append(child)
    return new_children

model = mujoco.MjModel.from_xml_path("mujoco/hexapod.xml")
mujoco.mj_saveLastXML("mujoco/hexapod_pinocchio.xml", model)

tree = ET.parse("mujoco/hexapod_pinocchio.xml")
root = tree.getroot()
worldbody = root.find("worldbody")
worldbody[:] = flatten_frames(worldbody)
tree.write("mujoco/hexapod_pinocchio.xml", encoding="utf-8")