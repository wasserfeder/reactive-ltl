'''
Created on May 14, 2014

@author: Cristian
'''

from base import Point


def ConfigurationSE2(Point):
    '''
    '''
    __slots__ = ()
    
    @property
    def x(self):
        return self.coords[0]
    @property
    def y(self):
        return self.coords[1]
    @property
    def theta(self):
        return self.coords[2]
    
    def __len__(self):
        return 3

def ConfigurationSE3(Point):
    '''
    '''
    __slots__ = ()
    
    @property
    def x(self):
        return self.coords[0]
    @property
    def y(self):
        return self.coords[1]
    @property
    def z(self):
        return self.coords[2]
    @property
    def roll(self):
        return self.coords[3]
    @property
    def pitch(self):
        return self.coords[4]
    @property
    def yaw(self):
        return self.coords[5]
    
    def __len__(self):
        return 6

if __name__ == '__main__':
    pass