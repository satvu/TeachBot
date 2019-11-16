import unittest
import kinematics

class TestKinematicMethods(unittest.TestCase):

    def test_forward_analytical_0(self):
        ur5e_params = [0.1625, -0.425, -0.3992, 0.1333, 0.0997, 0.0996]
        kin_solver = Kinematics(ur5e_params)
        forward_result = kin_solver.forward([])
        self.assertTrue([], forward_result)

    def test_forward_analytical_1(self):
        ur5e_params = [0.1625, -0.425, -0.3992, 0.1333, 0.0997, 0.0996]
        kin_solver = Kinematics(ur5e_params)
        forward_result = kin_solver.forward([])
        self.assertTrue([], forward_result)
    
    def test_forward_analytical_2(self):
        ur5e_params = [0.1625, -0.425, -0.3992, 0.1333, 0.0997, 0.0996]
        kin_solver = Kinematics(ur5e_params)
        forward_result = kin_solver.forward([])
        self.assertTrue([], forward_result)
    
    def test_inverse_analytical_0(self):
        return 
    
    def test_inverse_analytical_1(self):
        return 
    
    def test_inverse_analytics_outside_bounds(self):
        return 

    def test_inverse_analytical_singular(self):
        return 


if __name__ == '__main__':
    unittest.main()