#!/usr/bin/env python3

import unittest

class TestInitialization(unittest.TestCase):

    def test_nothing(self):
        self.assertTrue(True)


PKG = 'ros_web_client'
NAME = 'test_initialization'
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestInitialization)
