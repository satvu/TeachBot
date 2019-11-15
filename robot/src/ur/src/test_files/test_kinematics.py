import unittest
import kinematics

class TestKinematicMethods(unittest.TestCase):

    def test_forward_analytical(self):
        kin_solver = Kinematics([0.1625, -0.425, -0.3992, 0.1333, 0.0997, 0.0996])
        forward_result = kin_solver.forward([])

    def test_isupper(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())

    def test_split(self):
        s = 'hello world'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)

if __name__ == '__main__':
    unittest.main()