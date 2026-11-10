"""Load a manifest + its OBJ parts and evaluate forward kinematics (shared by validator and renderer)."""
import json
import math
import os

import numpy as np

import geom as g


class RoverModel:
    def __init__(self, manifest_path, manifest=None):
        self.path = os.path.abspath(manifest_path)
        self.dir = os.path.dirname(self.path)
        self.m = manifest if manifest is not None else json.load(open(self.path))
        self.parts = {p["name"]: p for p in self.m["render"]["parts"]}
        self.base = self.m["base_frame"]["name"]
        self._mesh = {}
        self.load_errors = {}

    def mesh(self, name):
        if name not in self._mesh:
            p = self.parts[name]
            try:
                self._mesh[name], _ = g.read_obj(os.path.join(self.dir, p["file"]))
            except Exception as e:  # noqa: BLE001
                self.load_errors[name] = str(e)
                self._mesh[name] = g.Mesh()
        return self._mesh[name]

    def joint_T(self, name, state):
        p = self.parts[name]
        j = p.get("joint")
        if not j or j.get("type") == "fixed" or name not in state:
            return np.eye(4)
        q = state[name] - j.get("authored_value", 0.0)
        ax = np.array(j.get("axis", [0, 0, 1]), float)
        ax = ax / np.linalg.norm(ax)
        if j["type"] in ("revolute", "continuous"):
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
            return g.make_T(R)
        return g.make_T(None, ax * q)

    def frame_T(self, name, state=None):
        """Pose of a part frame (or the base) in base coordinates."""
        state = state or {}
        if name == self.base or name not in self.parts:
            return np.eye(4)
        p = self.parts[name]
        return self.frame_T(p["parent"], state) @ g.T_from_pose(p["origin"]) @ self.joint_T(name, state)

    def world_mesh(self, name, state=None, T_world=None):
        T = self.frame_T(name, state)
        if T_world is not None:
            T = T_world @ T
        return self.mesh(name).transformed(T)

    def pose_of_frame(self, parent, origin, state=None, T_world=None):
        T = self.frame_T(parent, state) @ g.T_from_pose(origin)
        return T if T_world is None else T_world @ T

    def face(self, name):
        for f in self.m.get("attachments", {}).get("docking_faces", []):
            if f["name"] == name:
                return f
        raise KeyError(name)

    def face_T(self, name):
        return g.T_from_pose(self.face(name)["origin"])
