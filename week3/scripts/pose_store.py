#!/usr/bin/env python3

import os
import pickle


class PoseStore:
    def __init__(self, filename):
        self.filename = filename
        self.poses = {}
        self.load()

    def load(self):
        if os.path.exists(self.filename):
            with open(self.filename, 'rb') as f:
                self.poses = pickle.load(f)

    def save(self):
        with open(self.filename, 'wb') as f:
            pickle.dump(self.poses, f)

    def add(self, name, pose):
        self.poses[name] = pose
        self.save()

    def get(self, name):
        return self.poses.get(name)

    def delete(self, name):
        if name in self.poses:
            del self.poses[name]
            self.save()

    def overwrite(self, name, pose):
        if name in self.poses:
            self.poses[name] = pose
            self.save()

    def list(self):
        return self.poses.keys()
