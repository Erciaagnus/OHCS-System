#!/usr/bin/env python3
import queue
class User:
    def __init__(self):
        # Hi-Level Policy
        self.user_id = None
        self.pose = None
        self.request_time = None
    def update_request(self):
        self.request = None
class Charger:
    def __init__(self):
        # Hi-Level Policy
        self.charger_id = None
        self.waiting_queue = queue
        self.pose = None
