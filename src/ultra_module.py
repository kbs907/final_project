#! /usr/bin/env python

class UltraModule:
    ultra_data = 0
    
    def set_data(self, data):
        self.ultra_data = data.data
        
    def get_data(self):
        return self.ultra_data