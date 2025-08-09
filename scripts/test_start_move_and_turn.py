#!/usr/bin/env python3

import unittest
import rospy
import rostest
from std_srvs.srv import Trigger, TriggerResponse


class TestMoveAndTurnService(unittest.TestCase):
    def test_start_move_and_turn_service(self):
        # Create the service proxy
        service = rospy.ServiceProxy('/start_move_and_turn', Trigger)

        # Call the service and check the response
        response = service()
        self.assertTrue(response.success)
        self.assertEqual(response.message, "Move and Turn Started")

        # Close the service proxy to release resources
        service.close()


if __name__ == '__main__':
    rospy.init_node('test_move_and_turn_service')
    rospy.wait_for_service('/start_move_and_turn')

    # Run the test with rostest
    rostest.rosrun('move_and_turn', 'test_move_and_turn_service',
                   TestMoveAndTurnService)
